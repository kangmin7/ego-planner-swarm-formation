import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drone_configs = [
        {'drone_id': 0, 'target_y': -14.0, 'target_z': 2.0},
        {'drone_id': 1, 'target_y': 2.0, 'target_z': 2.0},
    ]

    ld = LaunchDescription()

    for cfg in drone_configs:
        i = cfg['drone_id']
        px4_ns = '' if i == 0 else f'/px4_{i}'
        lidar_cloud_topic = (
            f'/world/obstacles_swarm/model/x500_lidar360_{i}/link/link/sensor/lidar360/scan/points'
        )

        drone_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ego_planner'), 'launch', 'px4_single_run.launch.py')),
            launch_arguments={
                'drone_id': str(i),
                'px4_ns': px4_ns,
                'target_system': str(i + 1),
                'lidar_cloud_topic': lidar_cloud_topic,
                'map_size_x': str(110.0),
                'map_size_y': str(65.0),
                'map_size_z': str(5.0),
                'target_x': str(50.0),
                'target_y': str(cfg['target_y']),
                'target_z': str(cfg['target_z']),
                'emergency_time': str(2.0),
                'swarm_clearance': str(1.5),
                'max_vel': str(0.6),
                'max_acc': str(1.2),
                'virtual_ceil_height': str(4.3),
            }.items()
        )
        ld.add_action(drone_launch)

    return ld
