#!/usr/bin/env python3
"""Converts ego_planner's PositionCommand stream into PX4 offboard setpoints.

Subscribes `/position_cmd` (quadrotor_msgs/PositionCommand, ENU, published
by traj_server at 100 Hz once a trajectory has been received) and republishes
it as `px4_msgs/TrajectorySetpoint` (NED) on `/fmu/in/trajectory_setpoint`.
Streams `OffboardControlMode` at >2 Hz from startup, and once enough
heartbeats have gone out, arms and switches to OFFBOARD using the same
VehicleCommand sequence proven in px4_llm_control/mission_executor.py
(force-arm in SITL via param2=21196); retries every RETRY_TICKS until PX4
confirms ARMED.

State machine (GROUNDED -> TAKEOFF -> TRACKING):
PX4's own takeoff thrust ramp (MPC_TKO_RAMP_T) only applies in AUTO_TAKEOFF
mode, not raw OFFBOARD. ego_planner replans continuously from t=0 (even while
still disarmed on the ground) toward the far-away goal, so by the time arming
completes the planner's live PositionCommand can already carry significant
velocity. Forwarding that directly as the first setpoint after liftoff is a
large, discontinuous step for the position controller from a standing start,
which on this airframe produced a visible bad/aggressive climb. So after
arming we first hold a simple in-place vertical climb to TAKEOFF_ALT_M (no
planner input) and only start forwarding `/position_cmd` once that's reached.
"""
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,
    VehicleOdometry, VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand

from px4_bridge.frame_transforms import enu_to_ned_position, enu_to_ned_velocity, enu_yaw_to_ned_yaw

HEARTBEAT_TICKS = 30      # 1.5 s of OffboardControlMode before arm + offboard, matches mission_executor
RETRY_TICKS = 40          # resend arm + offboard every 2 s until PX4 confirms ARMED
TICK_HZ = 20.0            # higher rate keeps the per-tick position-setpoint step small as max_vel grows

# Hold 0.5 m ABOVE the 2.0 m cruise altitude: the handoff to the planner's
# first trajectory pitches the vehicle over hard, and the resulting altitude
# dip (~0.5-0.7 m observed) must not reach the near-ground lidar blind band /
# pillar-base region. Handing off at cruise altitude exactly (1.5, with the
# virtual floor at 1.0) left no margin and lost a drone to a ground-level
# pillar strike right after departure.
TAKEOFF_ALT_M = 3.0       # climb-in-place target altitude before handing off to the planner
                          # (estimated; gz ground truth showed the baro-referenced height
                          # reads ~0.65 m HIGH in flight, so the whole flight band is
                          # shifted up to keep true altitude safely off the ground)
TAKEOFF_TOLERANCE_M = 0.3

# Don't arm until the EKF height estimate has been quiet on the ground for
# this long (ticks at TICK_HZ). Arming the instant preflight passes -- on a
# barely-converged EKF -- produced two distinct wrecks: a runaway climb to
# 22 m (controller chasing a drifting height reference; PX4 had logged
# "height estimate not stable" / "vertical velocity unstable" moments
# earlier) and a takeoff that flew itself into the ground while its estimate
# read 2.2 m.
EKF_SETTLE_TICKS = 60     # 3 s of |vz| < 0.15 and |z| < 0.5 while grounded

# A distance gate used to live here ("only hand off once the live setpoint is
# near the drone"), added after liftoff produced a real impact with a pillar.
# But ego_planner's trajectory clock keeps advancing in real time on every
# REPLAN_TRAJ cycle regardless of whether anything is actually flying it --
# so if TAKEOFF ever withholds tracking even briefly while the gate isn't
# satisfied, the planned position drifts further every cycle, the gate gets
# harder to satisfy, never recovers, and TAKEOFF never completes (observed:
# one drone stuck climbing/hovering forever, never reaching TRACKING). The
# actual fix for the liftoff-impact bug is upstream of this file: gating
# ego_replan_fsm's *first* trajectory generation on real altitude
# (fsm/min_takeoff_height) so the trajectory's t=0 lines up with roughly when
# TAKEOFF here completes, instead of running from node startup. With that in
# place, altitude alone is sufficient for handoff again.


class State(Enum):
    GROUNDED = auto()   # disarmed, holding ground position, (re)trying to arm + engage offboard
    TAKEOFF = auto()    # armed, climbing straight up in place to TAKEOFF_ALT_M
    TRACKING = auto()   # at altitude, forwarding ego_planner's PositionCommand


class OffboardBridge(Node):

    def __init__(self):
        super().__init__('px4_offboard_bridge')

        # PX4 SITL instance 0 publishes un-namespaced /fmu/... topics; every
        # other instance i is namespaced /px4_i/fmu/... -- this lets one bridge
        # node per drone target the right instance in swarm mode.
        self.declare_parameter('px4_ns', '')
        px4_ns = self.get_parameter('px4_ns').value

        # PX4 SITL instance i has MAV_SYS_ID == i+1 and ignores VehicleCommands
        # addressed to a different target_system (commander only acts on
        # commands for its own system id, or broadcast id 0) -- target_system
        # must therefore match the instance this bridge talks to, not always 1.
        self.declare_parameter('target_system', 1)
        self._target_system = self.get_parameter('target_system').value

        # Counterpart of odom_cloud_bridge's spawn_offset_enu: the planner
        # commands positions in the shared world frame, PX4 expects them in
        # its own spawn-origin local frame, so subtract the spawn position
        # (world ENU) before the ENU->NED conversion. Keep both parameters
        # identical for a given drone.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self._spawn_offset_enu = (
            self.get_parameter('spawn_x').value,
            self.get_parameter('spawn_y').value,
            0.0,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_ocm = self.create_publisher(OffboardControlMode, f'{px4_ns}/fmu/in/offboard_control_mode', px4_qos)
        self._pub_tsp = self.create_publisher(TrajectorySetpoint, f'{px4_ns}/fmu/in/trajectory_setpoint', px4_qos)
        self._pub_cmd = self.create_publisher(VehicleCommand, f'{px4_ns}/fmu/in/vehicle_command', px4_qos)

        # vehicle_odometry, NOT vehicle_local_position: the latter silently
        # fails to stream on a large fraction of multi-instance boots, leaving
        # every consumer here (settle gate, takeoff check, carrot limiter)
        # running on a zero-frozen message (runs 80/81 runaways). Odometry is
        # the same EKF solution and streams reliably on every boot.
        self.create_subscription(
            VehicleOdometry, f'{px4_ns}/fmu/out/vehicle_odometry', self._cb_odom, px4_qos)
        self.create_subscription(
            VehicleStatus, f'{px4_ns}/fmu/out/vehicle_status_v1', self._cb_status, px4_qos)
        self.create_subscription(PositionCommand, '/position_cmd', self._cb_position_cmd, 10)

        self._local_pos = VehicleLocalPosition()
        self._status = VehicleStatus()
        self._last_arming_state = None
        self._last_nav_state = None

        self._have_position_cmd = False
        self._setpoint_ned = (0.0, 0.0, 0.0)     # (x, y, z) NED, from the planner
        self._velocity_ned = (0.0, 0.0, 0.0)     # (vx, vy, vz) NED, from the planner
        self._yaw_ned = None

        self._state = State.GROUNDED
        self._takeoff_xy_yaw = (0.0, 0.0, float('nan'))
        self._hb_count = 0
        self._ekf_settle_ticks = 0

        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info('px4_offboard_bridge ready — waiting for /position_cmd')

    def _cb_odom(self, msg: VehicleOdometry):
        lp = self._local_pos
        lp.x, lp.y, lp.z = float(msg.position[0]), float(msg.position[1]), float(msg.position[2])
        lp.vz = float(msg.velocity[2])
        qw, qx, qy, qz = (float(v) for v in msg.q)
        lp.heading = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _cb_status(self, msg: VehicleStatus):
        self._status = msg
        if msg.arming_state != self._last_arming_state or msg.nav_state != self._last_nav_state:
            self.get_logger().info(f'PX4: arming_state={msg.arming_state}, nav_state={msg.nav_state}')
            self._last_arming_state = msg.arming_state
            self._last_nav_state = msg.nav_state
        if msg.arming_state != VehicleStatus.ARMING_STATE_ARMED and self._state != State.GROUNDED:
            self.get_logger().warn(f'{self._state.name} -> GROUNDED (disarmed)')
            self._state = State.GROUNDED
            self._hb_count = 0

    def _cb_position_cmd(self, msg: PositionCommand):
        self._setpoint_ned = enu_to_ned_position(msg.position.x - self._spawn_offset_enu[0],
                                                 msg.position.y - self._spawn_offset_enu[1],
                                                 msg.position.z - self._spawn_offset_enu[2])
        self._velocity_ned = enu_to_ned_velocity(msg.velocity.x, msg.velocity.y, msg.velocity.z)
        self._yaw_ned = enu_yaw_to_ned_yaw(msg.yaw)
        self._have_position_cmd = True

    def _ts(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_cmd(self, command: int, **kw):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(kw.get('p1', 0))
        msg.param2 = float(kw.get('p2', 0))
        msg.target_system = self._target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._ts()
        self._pub_cmd.publish(msg)

    def _arm(self):
        # p2=21196.0 forces arm in SITL regardless of pre-flight check failures
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0, p2=21196.0)

    def _engage_offboard(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)

    def _send_setpoint(self, x, y, z, yaw, vx=None, vy=None, vz=None):
        tsp = TrajectorySetpoint()
        tsp.position = [float(x), float(y), float(z)]
        if vx is not None:
            tsp.velocity = [float(vx), float(vy), float(vz)]
        tsp.yaw = float('nan') if yaw is None or math.isnan(yaw) else float(yaw)
        tsp.timestamp = self._ts()
        self._pub_tsp.publish(tsp)

    def _tick(self):
        ocm = OffboardControlMode()
        ocm.position = True
        ocm.timestamp = self._ts()
        self._pub_ocm.publish(ocm)

        if self._state == State.GROUNDED:
            self._s_grounded()
        elif self._state == State.TAKEOFF:
            self._s_takeoff()
        elif self._state == State.TRACKING:
            self._s_tracking()

    def _s_grounded(self):
        # hold current position while arming/offboard is being engaged
        yaw = self._local_pos.heading if not math.isnan(self._local_pos.heading) else None
        self._send_setpoint(self._local_pos.x, self._local_pos.y, self._local_pos.z, yaw)

        if self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self._takeoff_xy_yaw = (self._local_pos.x, self._local_pos.y, self._local_pos.heading)
            self.get_logger().info(f'armed -> climbing in place to {TAKEOFF_ALT_M} m')
            self._state = State.TAKEOFF
            return

        # EKF settle gate (see EKF_SETTLE_TICKS above): hold off arming until
        # the height estimate is demonstrably stable on the ground.
        if abs(self._local_pos.vz) < 0.15 and abs(self._local_pos.z) < 0.5:
            self._ekf_settle_ticks += 1
        else:
            self._ekf_settle_ticks = 0
        if self._ekf_settle_ticks < EKF_SETTLE_TICKS:
            return
        if self._ekf_settle_ticks == EKF_SETTLE_TICKS:
            self.get_logger().info('EKF height stable -> proceeding to arm')

        self._hb_count += 1
        if self._hb_count >= HEARTBEAT_TICKS and self._hb_count % RETRY_TICKS == 0:
            self.get_logger().info('arming and engaging offboard')
            self._engage_offboard()
            self._arm()

    def _s_takeoff(self):
        x, y, _ = self._takeoff_xy_yaw
        target_z = -TAKEOFF_ALT_M
        # yaw=None (free yaw) -- the EKF heading estimate is often still noisy/transient
        # right at the moment of arming, so committing to it as a fixed target risked
        # commanding an aggressive, unnecessary yaw correction during the climb.
        self._send_setpoint(x, y, target_z, None)

        # One-sided check: hand off once the climb has reached altitude.
        # The symmetric |z - target| < tol version deadlocked when the
        # GPS-referenced height settled ~0.3 m ABOVE the setpoint (run 40:
        # two drones hovered at 2.79-2.80 m forever, never triggering their
        # planners, while the third flew the mission alone).
        if -self._local_pos.z >= TAKEOFF_ALT_M - TAKEOFF_TOLERANCE_M:
            if not self._have_position_cmd:
                return
            self.get_logger().info('takeoff complete -> tracking ego_planner')
            self._state = State.TRACKING

    def _s_tracking(self):
        if not self._have_position_cmd:
            x, y, _ = self._takeoff_xy_yaw
            self._send_setpoint(x, y, -TAKEOFF_ALT_M, None)
            return
        # Position-only, matching the proven mission_executor.py pattern -- mixing a
        # velocity feedforward into the same setpoint as position caused a sustained
        # roll/pitch oscillation on this airframe (observed via telemetry), most likely
        # the position controller and the velocity feedforward fighting each other.
        x, y, z = self._setpoint_ned
        # Carrot limiter: never command a point further than 2.5 m from where
        # the vehicle actually is. Planner-side bugs (a trajectory whose clock
        # raced ahead of a physically held-up drone) have produced 9 m setpoint
        # jumps that sent the vehicle on over-speed dashes into the ground;
        # this is the last line of defense against any such discontinuity.
        # Horizontal-only: scaling z toward the vehicle neuters the altitude
        # hold exactly while it chases a distant carrot, and it sinks out of
        # the sky (run 66). The commanded altitude always applies in full.
        dx = x - self._local_pos.x
        dy = y - self._local_pos.y
        d = math.sqrt(dx * dx + dy * dy)
        if d > 2.5:
            s = 2.5 / d
            x = self._local_pos.x + dx * s
            y = self._local_pos.y + dy * s
        self._send_setpoint(x, y, z, self._yaw_ned)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
