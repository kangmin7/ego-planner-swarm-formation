import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # PX4 SITL instance 0 publishes un-namespaced /fmu/... topics; instance i>=1
    # is namespaced /px4_i/fmu/.... drone_id == PX4 instance number here, so
    # each drone's pair of px4_bridge nodes targets the matching SITL instance.
    # Each drone's Gazebo lidar sensor is similarly scoped per spawned model
    # instance (x500_lidar360_<i>) once the model.sdf <topic> override was
    # removed -- see Tools/simulation/gz/models/lidar360/model.sdf.
    #
    # Drones are spread along Y (8 m apart) and fly straight down the pillar
    # field (x:[2.3,39.9]) in parallel lanes to a shared far-side target row,
    # past the last pillar -- ego_planner's swarm coordination (broadcast
    # bspline on a topic shared by all drone_id's) handles any inter-drone
    # conflicts if a drone has to dodge sideways into a neighboring lane.
    # 3 m spacing was tried first and was too tight: the distinctive-trajectory
    # avoidance optimizer hit "Can't find the new base points at the opposite
    # within the threshold" and diverged, sending two of three drones to wild
    # setpoints (tens of meters off-map, one deep "underground" in NED z).
    # 8 m Y-only spacing alone still triggered "Suddenly discovered obstacles"
    # emergency loops on the inner lanes -- the rebound optimizer, looking for
    # a way around a nearby teammate, found a path hugging near-zero altitude
    # (z~0.04m), exploiting the same lidar-vertical-sensor-gap issue fixed
    # earlier for the local-target fallback (16 scan rings leave gaps a naive
    # search reads as "free" right at a pillar's base). Staggering altitude
    # per lane too gives the optimizer a second axis of separation so it
    # doesn't need to dive toward the ground to avoid a teammate. Even with
    # that, the default emergency_time=1.0/swarm_clearance=0.5 (tuned against
    # the idealized point-mass so3 simulator) were too tight for real PX4
    # tracking lag/jitter and caused brief "Suddenly discovered obstacles"
    # loops; widening both (2.0 / 1.5) let those resolve into a normal
    # REPLAN_TRAJ/EXEC_TRAJ cycle instead of getting stuck.
    drone_configs = [
        {'drone_id': 0, 'spawn_y': -8.0, 'target_y': -8.0, 'target_z': 1.0},
        {'drone_id': 1, 'spawn_y': 0.0, 'target_y': 0.0, 'target_z': 1.8},
        {'drone_id': 2, 'spawn_y': 8.0, 'target_y': 8.0, 'target_z': 2.6},
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
                # PX4 SITL instance i has MAV_SYS_ID == i+1.
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
            }.items()
        )
        ld.add_action(drone_launch)

    return ld
