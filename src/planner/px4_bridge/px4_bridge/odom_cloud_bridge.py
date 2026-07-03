#!/usr/bin/env python3
"""Bridges PX4 odometry and the SITL lidar point cloud into ego_planner.

Subscribes `/fmu/out/vehicle_odometry` (px4_msgs/VehicleOdometry, NED
position + FRD-body-to-NED-world quaternion) and republishes it as
`nav_msgs/Odometry` in ENU/FLU on `~/odom` (-> remap to `grid_map/odom` /
`odom_world`).

Also subscribes the Gazebo-bridged lidar point cloud (still in the sensor's
own body-fixed frame) and, using the latest converted pose, rotates +
translates each point into world-ENU before republishing on `~/cloud_world`
(-> remap to `grid_map/cloud`) — GridMap::cloudCallback assumes points are
already in world frame, so this step plays the role the original (non-PX4)
flow's pcl_render_node played.

Known simplification: the lidar's small mounting offset from the vehicle
body origin (0/0/0.18 m, see PX4-Autopilot's x500_lidar360 model) is
ignored — points are transformed as if the sensor were at the body origin.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from px4_bridge.frame_transforms import (
    frd_ned_quat_to_flu_enu_quat,
    ned_to_enu_position,
    ned_to_enu_velocity,
    quat_to_rotation_matrix,
)


class OdomCloudBridge(Node):

    def __init__(self):
        super().__init__('px4_odom_cloud_bridge')

        self.declare_parameter('lidar_cloud_topic', '/lidar360/points')
        lidar_cloud_topic = self.get_parameter('lidar_cloud_topic').value

        # PX4 SITL instance 0 publishes un-namespaced /fmu/... topics; every
        # other instance i is namespaced /px4_i/fmu/... -- this lets one bridge
        # node per drone target the right instance in swarm mode.
        self.declare_parameter('px4_ns', '')
        px4_ns = self.get_parameter('px4_ns').value

        # Drop lidar returns closer than this (sensor-frame range, before the
        # world transform) -- excludes self/ground hits near the sensor's own
        # ~0.5 m minimum range. See _cb_cloud for why this matters.
        self.declare_parameter('min_obstacle_range', 0.8)
        self._min_obstacle_range = self.get_parameter('min_obstacle_range').value

        # PX4's EKF local origin sits at the vehicle's own spawn point, so in
        # multi-vehicle runs each instance reports positions in its own frame.
        # Cross-drone couplings (mutual avoidance, formation cost) need one
        # shared world frame: add this drone's spawn position (world ENU) to
        # every pose before republishing. Leave (0,0,0) if PX4 odometry is
        # already world-referenced in your setup.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self._spawn_offset_enu = np.array([
            self.get_parameter('spawn_x').value,
            self.get_parameter('spawn_y').value,
            0.0,
        ])

        # Neighbor-drone lidar returns must NOT reach the grid map: mutual
        # avoidance runs on broadcast trajectories, not the map, so a painted
        # neighbor (a ~1 m inflated column that jumps to a new position every
        # scan) is pure poison -- any drone flying behind another storms with
        # "Suddenly discovered obstacles" emergency stops against its
        # formation partners (observed: 87 events on the trailing drone vs 0
        # on the leading ones). Upstream ego-planner-swarm removes other
        # drones from sensor data the same way (drone_detect). Points within
        # neighbor_filter_radius of any other member's last known world
        # position are dropped; the radius covers the airframe plus one
        # scan-interval of neighbor motion at max_vel.
        self.declare_parameter('drone_id', -1)
        self.declare_parameter('formation_num', 0)
        self.declare_parameter('neighbor_filter_radius', 1.0)
        drone_id = self.get_parameter('drone_id').value
        formation_num = self.get_parameter('formation_num').value
        self._neighbor_filter_radius = self.get_parameter('neighbor_filter_radius').value
        self._neighbor_pos = {}
        if drone_id >= 0:
            for j in range(formation_num):
                if j == drone_id:
                    continue
                self.create_subscription(
                    Odometry, f'/drone_{j}_px4_odom',
                    lambda m, j=j: self._neighbor_pos.__setitem__(
                        j, np.array([m.pose.pose.position.x,
                                     m.pose.pose.position.y,
                                     m.pose.pose.position.z])),
                    10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_odom = self.create_publisher(Odometry, '~/odom', 10)
        self._pub_cloud = self.create_publisher(PointCloud2, '~/cloud_world', 10)

        self.create_subscription(
            VehicleOdometry, f'{px4_ns}/fmu/out/vehicle_odometry', self._cb_odom, px4_qos)
        self.create_subscription(
            PointCloud2, lidar_cloud_topic, self._cb_cloud, 10)

        self._have_pose = False
        self._position_enu = np.zeros(3)
        self._rotation_enu = np.eye(3)
        self._quat_enu = (1.0, 0.0, 0.0, 0.0)
        self._velocity_enu = (0.0, 0.0, 0.0)

        self.get_logger().info(
            f'px4_odom_cloud_bridge ready — px4_ns: "{px4_ns}", lidar cloud topic: {lidar_cloud_topic}')

    def _cb_odom(self, msg: VehicleOdometry):
        x, y, z = ned_to_enu_position(*msg.position)
        vx, vy, vz = ned_to_enu_velocity(*msg.velocity)
        q = frd_ned_quat_to_flu_enu_quat(msg.q)

        x, y, z = np.array([x, y, z]) + self._spawn_offset_enu

        self._position_enu = np.array([x, y, z])
        self._velocity_enu = (vx, vy, vz)
        self._quat_enu = q
        self._rotation_enu = quat_to_rotation_matrix(q)
        self._have_pose = True

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)
        self._pub_odom.publish(odom)

    def _cb_cloud(self, msg: PointCloud2):
        if not self._have_pose:
            return

        # read_points_numpy requires every field in the cloud to share one dtype, which
        # PointCloudPacked clouds (mixed float32/uint32 fields) violate -- read structured,
        # then pull out just x/y/z as a plain float32 (N, 3) array.
        structured = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if structured.shape[0] == 0:
            return
        points = point_cloud2.structured_to_unstructured(structured, dtype=np.float32)
        # out-of-range lidar rays come back as +-inf rather than NaN, so skip_nans alone
        # doesn't remove them
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return

        # GridMap::cloudCallback inflates every point it receives with no
        # ground/self-range filtering of its own (unlike the original
        # depth-camera pipeline, which never saw points this close). The
        # 360 lidar's downward rays return self/ground hits clustering at
        # its ~0.5 m minimum range, especially during liftoff and low-altitude
        # emergency-stop holds; left unfiltered these get inflated straight
        # into the occupancy grid under the drone's own position, causing
        # spurious "the drone is in obstacle" / emergency-stop loops.
        ranges = np.linalg.norm(points, axis=1)
        points = points[ranges > self._min_obstacle_range]
        if points.shape[0] == 0:
            return

        world_points = points @ self._rotation_enu.T + self._position_enu

        # Drop returns from formation partners (see __init__).
        for npos in self._neighbor_pos.values():
            d2 = np.sum((world_points - npos) ** 2, axis=1)
            world_points = world_points[d2 > self._neighbor_filter_radius ** 2]
        if world_points.shape[0] == 0:
            return

        out = point_cloud2.create_cloud_xyz32(msg.header, world_points.tolist())
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        self._pub_cloud.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
