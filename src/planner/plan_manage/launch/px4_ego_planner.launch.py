# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */
"""EGO-Planner on the same PX4 SITL x500 / gz world / depth camera as MIGHTY.

Copied from the mighty package (launch/px4_ego_planner.launch.py), where
scripts/px4_obstacle_test.py --planner ego starts it. The depth adapter is
this package's copy (scripts/ego_depth_adapter.py); the PX4 bridge
(px4_mighty_bridge.py) and the gz world/vehicle still come from the mighty
package, which must be in the sourced workspace.

  gz depth image -> ego_depth_adapter (restamped image + optical camera pose)
      -> EGO-Planner grid map (pose_type 1) -> planner -> traj_server
      -> PositionCommand -> px4_mighty_bridge -> /fmu/in/trajectory_setpoint
  /fmu/out/vehicle_odometry -> px4_mighty_bridge -> odometry + TF

EGO-Planner keeps its own tuning (advanced_param.launch.py); only the camera
intrinsics, the goal, the ceiling, the map extent and (for matched-limit
comparisons) max_vel / max_acc are set here. It flies to a preset
waypoint (flight_type 2) because its manual-goal mode forces z = 1.0; the
harness releases it after the bridge's takeoff (see min_takeoff_height).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# x500_depth_mighty's sensor: 320x240, 1.274 rad horizontal FOV, square pixels.
WIDTH, HEIGHT, HFOV = 320, 240, 1.274
FX = (WIDTH / 2.0) / math.tan(HFOV / 2.0)
DRONE_ID = '0'


def launch_setup(context, *args, **kwargs):
    ns = LaunchConfiguration('namespace').perform(context)
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    origin = [float(v) for v in arg('origin_offset').split(',')]
    goal = arg('goal').split(',')
    map_size = arg('map_size').split(',')
    prefix = f'/drone_{DRONE_ID}_'

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
        arguments=['/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '--ros-args', '--log-level', 'warn'],
        remappings=[('/depth_camera', f'/{ns}/depth_image'), ('/clock', '/gz/clock')],
        output='screen')

    depth_adapter = Node(
        package='ego_planner', executable='ego_depth_adapter.py', name='ego_depth_adapter',
        namespace=ns, output='screen',
        remappings=[('ego/depth', f'{prefix}depth_image'),
                    ('ego/camera_pose', f'{prefix}camera_pose')])

    px4_bridge = Node(
        package='mighty', executable='px4_mighty_bridge.py', name='px4_mighty_bridge',
        namespace=ns, output='screen',
        parameters=[{'origin_offset': origin,
                     'takeoff_altitude': float(arg('takeoff_altitude')),
                     'position_cmd_topic': f'{prefix}planning/pos_cmd'}],
        remappings=[('odom', f'{prefix}odom')])

    ego = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': DRONE_ID,
            'odometry_topic': 'odom',
            'depth_topic': 'depth_image',
            'camera_pose_topic': 'camera_pose',
            'cloud_topic': 'cloud',
            'use_depth_filter': 'True',     # the grid map only subscribes to depth with it
            'cx': str(WIDTH / 2.0), 'cy': str(HEIGHT / 2.0),
            'fx': str(FX), 'fy': str(FX),
            # Centred on the origin: the x extent must cover twice the goal's x.
            'map_size_x_': map_size[0], 'map_size_y_': map_size[1], 'map_size_z_': map_size[2],
            'max_vel': arg('max_vel'),
            'max_acc': arg('max_acc'),
            'flight_type': '2',
            'point_num': '1',
            'point0_x': goal[0], 'point0_y': goal[1], 'point0_z': goal[2],
            # Same band as MIGHTY's z_max 3.0 (px4_mighty.yaml).
            'virtual_ceil_height': '3.0',
            # advanced_param runs with fsm/realworld_experiment False, which
            # sets the start trigger itself: without this gate EGO-Planner
            # planned from the ground before the bridge's takeoff (measured).
            # It holds planning until the vehicle is up and settled.
            'min_takeoff_height': str(float(arg('takeoff_altitude')) - 0.2),
        }.items())

    traj_server = Node(
        package='ego_planner', executable='traj_server', name=f'drone_{DRONE_ID}_traj_server',
        output='screen',
        remappings=[('position_cmd', f'{prefix}planning/pos_cmd'),
                    ('planning/bspline', f'{prefix}planning/bspline')],
        parameters=[{'traj_server/time_forward': 1.0}])

    # EGO-Planner's grid map latches "depth lost" (and then EMERGENCY_STOP
    # once triggered) if no depth+pose pair arrives within odom_depth_timeout
    # (1 s) of starting; the depth stream needs a few seconds (camera TF, gz
    # sensor) - measured ~6 s. Start the planner once its sensors are live,
    # still well before the bridge's takeoff completes.
    # EGO-Planner stamps its map and markers in "world" (and some in "map");
    # it consumes our map-frame poses, so the two frames coincide.
    world_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='world_to_map',
        arguments=['--frame-id', 'world', '--child-frame-id', 'map'], output='log')

    return [bridge, depth_adapter, px4_bridge, world_tf,
            TimerAction(period=10.0, actions=[ego, traj_server])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='NX01'),
        DeclareLaunchArgument('origin_offset', default_value='0.0,0.0,0.0'),
        DeclareLaunchArgument('takeoff_altitude', default_value='2.0'),
        DeclareLaunchArgument('goal', default_value='20.0,0.0,2.0'),
        DeclareLaunchArgument('map_size', default_value='42.0,30.0,5.0'),
        # EGO-Planner's advanced_param defaults; MIGHTY flies v_max 2.0 too.
        DeclareLaunchArgument('max_vel', default_value='2.0'),
        DeclareLaunchArgument('max_acc', default_value='3.0'),
        OpaqueFunction(function=launch_setup),
    ])
