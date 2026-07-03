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
    # instance (x500_lidar360_<i>).
    #
    # FORMATION FLIGHT: the three drones fly as an equilateral triangle wedge
    # (side 4 m, leader in front) through the pillar field, held together by
    # the formation-similarity cost ported from ZJU-FAST-Lab/Swarm-Formation
    # (formation_type=2 selects the triangle in BsplineOptimizer::
    # setDesiredFormation -- spawn/target offsets below must stay consistent
    # with that shape). The similarity metric is scale/rotation invariant, so
    # the swarm may compress or rotate to slip between pillars and re-expand;
    # the hard lower bound on separation is still swarm_clearance.
    #
    # spawn_x/spawn_y MUST match the PX4_GZ_MODEL_POSE values used by
    # scripts/start_formation_sitl.sh -- the bridges use them to shift each
    # PX4 instance's spawn-origin local frame into the one shared world frame
    # (a mismatch silently corrupts the formation graph and mutual avoidance).
    #
    # Earlier parallel-lane tuning that still applies: default
    # emergency_time=1.0/swarm_clearance=0.5 (tuned against the idealized
    # point-mass so3 simulator) were too tight for real PX4 tracking
    # lag/jitter and caused "Suddenly discovered obstacles" loops; 2.0/1.5
    # lets those resolve into a normal REPLAN_TRAJ/EXEC_TRAJ cycle.
    # swarm_clearance=1.5 keeps the mutual-avoidance floor (2*1.5=3 m) below
    # the 4 m formation side so the two costs don't fight at the desired shape.
    side = 4.0
    depth = side * 0.866  # equilateral triangle height along x
    half = side / 2.0
    target_x = 50.0  # past the last pillar at x=39.9
    drone_configs = [
        {'drone_id': 0, 'spawn_x': 0.0,    'spawn_y': 0.0,
         'target_x': target_x,          'target_y': 0.0},
        {'drone_id': 1, 'spawn_x': -depth, 'spawn_y': -half,
         'target_x': target_x - depth,  'target_y': -half},
        {'drone_id': 2, 'spawn_x': -depth, 'spawn_y': half,
         'target_x': target_x - depth,  'target_y': half},
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
                'spawn_x': str(cfg['spawn_x']),
                'spawn_y': str(cfg['spawn_y']),
                'map_size_x': str(110.0),
                'map_size_y': str(65.0),
                'map_size_z': str(5.0),
                'target_x': str(cfg['target_x']),
                'target_y': str(cfg['target_y']),
                'target_z': str(2.0),
                'emergency_time': str(2.0),
                'swarm_clearance': str(1.5),
                'formation_type': str(2),
                'formation_num': str(3),
                # Tuning history: 10.0 kept the optimizer healthy but was too
                # weak to couple the swarm in transit (drone 0 flew to the
                # goal alone, separation hit 33 m, formation only re-formed on
                # arrival -- the SNL gradient saturates with distance, so the
                # weight must dominate goal pull while the stretch is still
                # small). 100.0 held a tight 4-5 m triangle through the first
                # third of the pillar field but overrode obstacle/mutual
                # avoidance around x~20: drones 0 and 1 crashed.
                'lambda_formation': str(30.0),
                # Slower than the 5.0 used for independent parallel lanes:
                # formation keeping adds coupling between the drones'
                # replans, and PX4 tracking lag at high speed shows up as
                # formation error the optimizer then chases. 3.0 still
                # crashed a follower mid-field (hard sideways dodge across
                # the leader's lane at max speed left no tracking margin).
                'max_vel': str(2.0),
                # 5.0 pitched the vehicle over hard enough at the
                # takeoff->planner handoff (and on mid-field dodges) to dump
                # ~0.5-0.7 m of altitude into the lidar-blind near-ground
                # band; one drone clipped a pillar base and was lost.
                'max_acc': str(3.0),
                # Must sit inside [TAKEOFF_ALT_M - TAKEOFF_TOLERANCE_M,
                # TAKEOFF_ALT_M] of offboard_bridge.py (2.5 / 0.3): planning
                # may only start once the bridge's climb is essentially done,
                # and the bridge hands off to planner tracking at 2.2.
                'min_takeoff_height': str(2.2),
                # Keep plans well below the 4.0 m pillar tops: with 4.3 the
                # optimizer escaped vertically to ~3.8 and the steep dive back
                # to cruise overshot into the ground/pillar band (run 20 loss).
                'virtual_ceil_height': str(3.0),
                # Fence off the lidar's near-ground blind band (map starts at
                # ground_height=0.5) so escape plans can't hug the floor.
                'virtual_floor_height': str(1.5),
            }.items()
        )
        ld.add_action(drone_launch)

    return ld
