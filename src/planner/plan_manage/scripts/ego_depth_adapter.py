#!/usr/bin/env python3
# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */
"""gz depth camera -> EGO-Planner's depth + camera-pose inputs.

Copied from the mighty package (scripts/ego_depth_adapter.py). Used by
launch/px4_ego_planner.launch.py to fly EGO-Planner through the same PX4 /
world / harness as MIGHTY (mighty's scripts/px4_obstacle_test.py --planner ego).

EGO-Planner's grid map (grid_map/pose_type 1) pairs each depth image with a
camera PoseStamped by timestamp (ApproximateTime). gz stamps images with sim
time while the TF from px4_mighty_bridge is on wall time, so each image is
restamped to its capture time on the wall clock (gz clock offset, as in
depth_image_to_cloud.py) and published together with the camera's optical
frame pose (z forward, x right, y down) at that same time. The image data is
unchanged: EGO-Planner converts 32FC1 depth to millimetres itself.
"""

import collections

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
import tf2_ros

# Columns: the optical frame's axes expressed in the FLU camera link frame.
R_FLU_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s)
    s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s)


class EgoDepthAdapter(Node):

    def __init__(self):
        super().__init__('ego_depth_adapter')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', '')       # '' = <ns>/depth_camera_link
        self.declare_parameter('sim_clock_topic', '/gz/clock')
        ns = self.get_namespace().strip('/')
        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value or f'{ns}/depth_camera_link'
        self.offsets = collections.deque(maxlen=50)  # wall - sim; smallest = least delayed
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub_depth = self.create_publisher(Image, 'ego/depth', 10)
        self.pub_pose = self.create_publisher(PoseStamped, 'ego/camera_pose', 10)
        self.create_subscription(Clock, self.get_parameter('sim_clock_topic').value,
                                 self.clock_cb, qos_profile_sensor_data)
        self.create_subscription(Image, 'depth_image', self.image_cb, qos_profile_sensor_data)

    def clock_cb(self, msg):
        wall = self.get_clock().now().nanoseconds * 1e-9
        self.offsets.append(wall - (msg.clock.sec + msg.clock.nanosec * 1e-9))

    def image_cb(self, msg):
        if not self.offsets:
            return
        capture = Time(nanoseconds=int(
            (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + min(self.offsets)) * 1e9))
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, capture)
        except tf2_ros.TransformException:
            try:  # capture slightly newer than the latest TF: use the latest
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.camera_frame, Time())
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f'no camera TF: {e}', throttle_duration_sec=5.0)
                return
        q, p = tf.transform.rotation, tf.transform.translation
        qw, qx, qy, qz = rot_to_quat(quat_to_rot(q.w, q.x, q.y, q.z) @ R_FLU_OPTICAL)

        pose = PoseStamped()
        pose.header.stamp = capture.to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x, p.y, p.z
        pose.pose.orientation.w, pose.pose.orientation.x = float(qw), float(qx)
        pose.pose.orientation.y, pose.pose.orientation.z = float(qy), float(qz)
        msg.header.stamp = pose.header.stamp
        self.pub_depth.publish(msg)
        self.pub_pose.publish(pose)


def main():
    rclpy.init()
    node = EgoDepthAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
