import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    drone_id = LaunchConfiguration('drone_id', default=0)
    drone_id_cmd = DeclareLaunchArgument('drone_id', default_value=drone_id, description='Drone ID')

    # Pillar field (obstacles_swarm.sdf, 127 pillars) spans x:[2.3,39.9] y:[-30,30];
    # drone spawns at (0,0). grid_map's map_origin_ is fixed at
    # (-map_size_x/2, -map_size_y/2, ...) -- i.e. centered on the world origin
    # (0,0,0), NOT re-centered on the drone -- so map_size_x must cover out to
    # the target, not just the pillar field, or the boundary silently clips
    # everything past it (looks like open space to the planner, but produces
    # invalid grid indices / "Coord2Index ... index=-1" once something tries to
    # use a point past the edge). target_x=50 (clear of the last pillar at
    # x=39.9) needs map_size_x >= 2*50=100; 110 leaves a margin.
    map_size_x = LaunchConfiguration('map_size_x', default=110.0)
    map_size_y = LaunchConfiguration('map_size_y', default=65.0)
    map_size_z = LaunchConfiguration('map_size_z', default=5.0)
    map_size_x_cmd = DeclareLaunchArgument('map_size_x', default_value=map_size_x, description='Map size along x')
    map_size_y_cmd = DeclareLaunchArgument('map_size_y', default_value=map_size_y, description='Map size along y')
    map_size_z_cmd = DeclareLaunchArgument('map_size_z', default_value=map_size_z, description='Map size along z')

    # Past the far end of the pillar field (last pillar at x=39.9), not in the
    # thick of it -- previous target (35,0,1.5) sat among the densest pillars.
    target_x = LaunchConfiguration('target_x', default=50.0)
    target_y = LaunchConfiguration('target_y', default=0.0)
    target_z = LaunchConfiguration('target_z', default=1.5)
    target_x_cmd = DeclareLaunchArgument('target_x', default_value=target_x, description='Target x (ENU)')
    target_y_cmd = DeclareLaunchArgument('target_y', default_value=target_y, description='Target y (ENU)')
    target_z_cmd = DeclareLaunchArgument('target_z', default_value=target_z, description='Target z (ENU)')

    lidar_cloud_topic = LaunchConfiguration('lidar_cloud_topic', default='/lidar360/points')
    lidar_cloud_topic_cmd = DeclareLaunchArgument(
        'lidar_cloud_topic', default_value=lidar_cloud_topic,
        description='Gazebo-bridged lidar PointCloud2 topic (body frame)')

    # PX4 SITL instance 0 publishes un-namespaced /fmu/... topics; every other
    # instance i is namespaced /px4_i/fmu/... -- swarm mode sets this per drone
    # (drone_id == PX4 instance number) so each pair of bridge nodes talks to
    # the right SITL instance.
    px4_ns = LaunchConfiguration('px4_ns', default='')
    px4_ns_cmd = DeclareLaunchArgument('px4_ns', default_value=px4_ns, description='PX4 topic namespace, e.g. /px4_1')

    # PX4 SITL instance i has MAV_SYS_ID == i+1; VehicleCommand.target_system
    # must match it or the instance's commander ignores the command.
    target_system = LaunchConfiguration('target_system', default=1)
    target_system_cmd = DeclareLaunchArgument(
        'target_system', default_value=target_system, description='PX4 MAV_SYS_ID of the target instance')

    emergency_time = LaunchConfiguration('emergency_time', default=1.0)
    swarm_clearance = LaunchConfiguration('swarm_clearance', default=0.5)
    emergency_time_cmd = DeclareLaunchArgument(
        'emergency_time', default_value=emergency_time, description='Emergency stop time window')
    swarm_clearance_cmd = DeclareLaunchArgument(
        'swarm_clearance', default_value=swarm_clearance, description='Min inter-drone clearance')

    max_vel = LaunchConfiguration('max_vel', default=1.0)
    max_acc = LaunchConfiguration('max_acc', default=2.0)
    max_vel_cmd = DeclareLaunchArgument('max_vel', default_value=max_vel, description='Max velocity')
    max_acc_cmd = DeclareLaunchArgument('max_acc', default_value=max_acc, description='Max acceleration')

    # Pillars in obstacles_swarm.sdf are 4.0 m tall; the default virtual_ceil_height
    # (2.9 m) sits below that, so the planner has no vertical escape route when
    # boxed in laterally -- it's forced to dodge purely sideways through a
    # uniformly dense field. Raising this lets it climb above the pillars as a
    # genuine option when stuck.
    virtual_ceil_height = LaunchConfiguration('virtual_ceil_height', default=2.9)
    virtual_ceil_height_cmd = DeclareLaunchArgument(
        'virtual_ceil_height', default_value=virtual_ceil_height, description='Virtual ceiling height')

    # offboard_bridge holds the drone in an in-place climb to ~1.5 m (TAKEOFF_ALT_M)
    # before it starts forwarding the planner's setpoints. Without this gate the
    # planner starts replanning from t=0 at node startup, several seconds before the
    # drone is physically off the ground -- see ego_replan_fsm.cpp's min_takeoff_height
    # comment for the full failure mode (planner's internal trajectory races ahead of
    # the still-grounded real drone). 1.0 m is comfortably below the bridge's 1.5 m
    # climb target so the planner is ready slightly before the bridge's handoff check.
    min_takeoff_height = LaunchConfiguration('min_takeoff_height', default=1.0)
    min_takeoff_height_cmd = DeclareLaunchArgument(
        'min_takeoff_height', default_value=min_takeoff_height,
        description='Min real altitude (odom z) before the planner starts its first trajectory')

    odom_topic = 'px4_odom'
    cloud_topic = 'px4_cloud_world'

    # Include advanced parameters (PX4 odometry + lidar point cloud, no depth filter)
    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ego_planner'), 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'cloud_topic': cloud_topic,
            'use_depth_filter': 'False',

            'max_vel': max_vel,
            'max_acc': max_acc,
            'planning_horizon': str(5.0),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            'emergency_time': emergency_time,
            'swarm_clearance': swarm_clearance,
            'virtual_ceil_height': virtual_ceil_height,
            'min_takeoff_height': min_takeoff_height,
        }.items()
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
        ],
        parameters=[
            {'traj_server/time_forward': 1.0}
        ]
    )

    odom_cloud_bridge_node = Node(
        package='px4_bridge',
        executable='odom_cloud_bridge',
        name=['drone_', drone_id, '_px4_odom_cloud_bridge'],
        output='screen',
        parameters=[
            {'lidar_cloud_topic': lidar_cloud_topic},
            {'px4_ns': px4_ns},
        ],
        remappings=[
            ('~/odom', ['drone_', drone_id, '_', odom_topic]),
            ('~/cloud_world', ['drone_', drone_id, '_', cloud_topic]),
        ],
    )

    offboard_bridge_node = Node(
        package='px4_bridge',
        executable='offboard_bridge',
        name=['drone_', drone_id, '_px4_offboard_bridge'],
        output='screen',
        parameters=[
            {'px4_ns': px4_ns},
            {'target_system': target_system},
        ],
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
        ],
    )

    def _make_lidar_bridge_node(context, *args, **kwargs):
        # ros_gz_bridge's parameter_bridge needs a YAML config file (positional
        # <topic>@type[gz_type] args were found unreliable in this setup -- see
        # repo history). Writing it under /tmp meant every environment reset
        # wiped it and required manually recreating it before each run. ~/.ros
        # is not /tmp and survives those resets, and regenerating it here on
        # every launch means it's always in sync with lidar_cloud_topic anyway.
        topic = lidar_cloud_topic.perform(context)
        did = drone_id.perform(context)
        config_dir = os.path.expanduser('~/.ros/px4_bridge_lidar_configs')
        os.makedirs(config_dir, exist_ok=True)
        config_path = os.path.join(config_dir, f'lidar_bridge_{did}.yaml')
        with open(config_path, 'w') as f:
            f.write(
                f'- ros_topic_name: "{topic}"\n'
                f'  gz_topic_name: "{topic}"\n'
                '  ros_type_name: "sensor_msgs/msg/PointCloud2"\n'
                '  gz_type_name: "gz.msgs.PointCloudPacked"\n'
                '  direction: GZ_TO_ROS\n'
            )
        return [Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=['drone_', did, '_lidar_bridge'],
            output='screen',
            parameters=[{'config_file': config_path}],
            additional_env={'GZ_VERSION': 'harmonic'},
        )]

    lidar_bridge_action = OpaqueFunction(function=_make_lidar_bridge_node)

    ld = LaunchDescription()
    ld.add_action(drone_id_cmd)
    ld.add_action(map_size_x_cmd)
    ld.add_action(map_size_y_cmd)
    ld.add_action(map_size_z_cmd)
    ld.add_action(target_x_cmd)
    ld.add_action(target_y_cmd)
    ld.add_action(target_z_cmd)
    ld.add_action(lidar_cloud_topic_cmd)
    ld.add_action(px4_ns_cmd)
    ld.add_action(target_system_cmd)
    ld.add_action(emergency_time_cmd)
    ld.add_action(swarm_clearance_cmd)
    ld.add_action(max_vel_cmd)
    ld.add_action(max_acc_cmd)
    ld.add_action(virtual_ceil_height_cmd)
    ld.add_action(min_takeoff_height_cmd)

    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(odom_cloud_bridge_node)
    ld.add_action(offboard_bridge_node)
    ld.add_action(lidar_bridge_action)

    return ld
