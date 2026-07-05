"""Forward gz ground-truth odometry into PX4 as external-vision odometry.

The model's OdometryPublisher plugin emits world-frame (ENU/FLU) odometry,
which ros_gz's parameter_bridge exposes as nav_msgs/Odometry on
/model/<model>/odometry. This node re-expresses it in the PX4 instance's
spawn-origin NED frame and publishes px4_msgs/VehicleOdometry on
fmu/in/vehicle_visual_odometry, where the EKF fuses it (EKF2_EV_CTRL=15,
EKF2_HGT_REF=3 in the airframe file).

Rationale: the simulated baro/GPS/IMU chain destabilizes the EKF on the dev
host (meter-scale height bias and position excursions under load); a
mocap-grade external-vision feed makes the state estimate exact so the
planning stack above it can be validated.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry


class TruthOdomBridge(Node):
    def __init__(self):
        super().__init__('truth_odom_bridge')

        self.declare_parameter('px4_ns', '')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        px4_ns = self.get_parameter('px4_ns').value
        self._sx = self.get_parameter('spawn_x').value
        self._sy = self.get_parameter('spawn_y').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            VehicleOdometry, f'{px4_ns}/fmu/in/vehicle_visual_odometry', px4_qos)
        self.create_subscription(Odometry, 'truth_odom', self._cb, 10)
        self.get_logger().info(
            f'truth_odom_bridge ready — px4_ns "{px4_ns}", spawn ({self._sx}, {self._sy})')

    def _cb(self, msg: Odometry):
        out = VehicleOdometry()
        t_us = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = t_us
        out.timestamp_sample = t_us

        p = msg.pose.pose.position
        # gz OdometryPublisher reports WORLD-frame pose (verified live in
        # run 60: a grounded off-origin drone reads its spawn coordinates).
        # PX4's local frame origin is the spawn, so subtract it. GPS/baro
        # fusion must stay disabled or the GPS frame drags the origin back
        # to the world origin (the run 58/59 confusion).
        ex, ey, ez = p.x - self._sx, p.y - self._sy, p.z
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = [float(ey), float(ex), float(-ez)]

        # orientation: FLU/ENU quaternion -> FRD/NED (q_ned = q_enu_ned * q * q_flu_frd)
        q = msg.pose.pose.orientation  # x, y, z, w (FLU in ENU)
        # ENU->NED world rotation: (w,x,y,z) = (0, sqrt2/2, sqrt2/2, 0)
        # FLU->FRD body rotation:  (w,x,y,z) = (0, 1, 0, 0)
        s = 0.7071067811865476
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        # left-multiply by world ENU->NED
        w1, x1, y1, z1 = (
            -s * qx - s * qy,
            s * qw + s * qz,
            s * qw - s * qz,
            s * qx - s * qy,
        )
        # right-multiply by body FLU->FRD (0,1,0,0)
        out.q = [
            float(-x1),
            float(w1),
            float(z1),
            float(-y1),
        ]

        # velocity: OdometryPublisher gives body-frame twist; PX4 accepts
        # velocity_frame BODY_FRD.
        v = msg.twist.twist.linear
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out.velocity = [float(v.x), float(-v.y), float(-v.z)]

        av = msg.twist.twist.angular
        out.angular_velocity = [float(av.x), float(-av.y), float(-av.z)]

        out.position_variance = [0.001, 0.001, 0.001]
        out.orientation_variance = [0.001, 0.001, 0.001]
        out.velocity_variance = [0.01, 0.01, 0.01]
        out.quality = 100

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TruthOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
