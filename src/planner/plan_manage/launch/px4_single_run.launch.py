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

    # This drone's spawn position in the shared world frame (ENU). PX4's EKF
    # origin is the spawn point, so the bridges use this to convert between
    # PX4-local and world coordinates -- required for anything that couples
    # drones' absolute positions (mutual avoidance, formation cost).
    spawn_x = LaunchConfiguration('spawn_x', default=0.0)
    spawn_y = LaunchConfiguration('spawn_y', default=0.0)
    spawn_x_cmd = DeclareLaunchArgument('spawn_x', default_value=spawn_x, description='Spawn x in world ENU')
    spawn_y_cmd = DeclareLaunchArgument('spawn_y', default_value=spawn_y, description='Spawn y in world ENU')

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

    formation_type = LaunchConfiguration('formation_type', default=0)
    lambda_formation = LaunchConfiguration('lambda_formation', default=0.0)
    formation_num = LaunchConfiguration('formation_num', default=0)
    formation_type_cmd = DeclareLaunchArgument(
        'formation_type', default_value=formation_type,
        description='Desired formation shape (0=none, 1=hexagon(7), 2=triangle(3))')
    lambda_formation_cmd = DeclareLaunchArgument(
        'lambda_formation', default_value=lambda_formation, description='Formation similarity cost weight')
    formation_num_cmd = DeclareLaunchArgument(
        'formation_num', default_value=formation_num,
        description='Formation member count for the takeoff barrier (0 disables)')
    formation_stretch_max = LaunchConfiguration('formation_stretch_max', default=-1.0)
    formation_nominal_leads = LaunchConfiguration('formation_nominal_leads', default='')
    formation_nominal_leads_cmd = DeclareLaunchArgument(
        'formation_nominal_leads', default_value=formation_nominal_leads,
        description='Comma-separated nominal along-path lead per member (m)')
    formation_stretch_max_cmd = DeclareLaunchArgument(
        'formation_stretch_max', default_value=formation_stretch_max,
        description='Max lead (m) over the laggiest member before the local-target horizon throttles (<=0 disables)')

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

    virtual_floor_height = LaunchConfiguration('virtual_floor_height', default=-1000.0)
    virtual_floor_height_cmd = DeclareLaunchArgument(
        'virtual_floor_height', default_value=virtual_floor_height,
        description='Mark map cells below this z occupied (default disabled)')

    # offboard_bridge holds the drone in an in-place climb to ~1.5 m (TAKEOFF_ALT_M)
    # before it starts forwarding the planner's setpoints. Without this gate the
    # planner starts replanning from t=0 at node startup, several seconds before the
    # drone is physically off the ground -- see ego_replan_fsm.cpp's min_takeoff_height
    # comment for the full failure mode (planner's internal trajectory races ahead of
    # the still-grounded real drone).
    #
    # Matched to the bridge's 1.5 m climb target (was 1.0 m, a 0.5 m head start) --
    # at higher max_vel that half-metre gap let the planner's trajectory clock run for
    # a second or two before handoff, so the very first forwarded setpoint could
    # already carry near-max_vel velocity instead of starting from rest, producing a
    # visible speed burst right as TRACKING begins. The bridge's handoff already waits
    # for _have_position_cmd on top of the altitude check (offboard_bridge.py
    # _s_takeoff), so matching the two altitudes exactly costs nothing -- the bridge
    # simply holds the in-place climb a little longer if the planner isn't ready yet.
    min_takeoff_height = LaunchConfiguration('min_takeoff_height', default=1.5)
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
            # Multi-topology search multiplies optimizer attempts in dense
            # clutter and precedes every -1008 line-search failure storm
            # (drones wedge mid-field cycling GEN_NEW_TRAJ, run 71). Single
            # topology is slightly less optimal but far more robust here.
            'use_distinctive_trajs': 'False',
            'flight_type': str(2),
            'point_num': str(1),
            'point0_x': target_x,
            'point0_y': target_y,
            'point0_z': target_z,
            'emergency_time': emergency_time,
            'swarm_clearance': swarm_clearance,
            'formation_type': formation_type,
            'lambda_formation': lambda_formation,
            'formation_num': formation_num,
            'formation_stretch_max': formation_stretch_max,
            'formation_nominal_leads': formation_nominal_leads,
            'virtual_ceil_height': virtual_ceil_height,
            'virtual_floor_height': virtual_floor_height,
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
            {'spawn_x': spawn_x},
            {'spawn_y': spawn_y},
            # Enables neighbor-return filtering: formation partners must not
            # be painted into the grid map (see odom_cloud_bridge.py).
            {'drone_id': drone_id},
            {'formation_num': formation_num},
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
            {'spawn_x': spawn_x},
            {'spawn_y': spawn_y},
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

    def _make_truth_bridge_nodes(context, *args, **kwargs):
        # Ground-truth odometry path (mocap stand-in): gz OdometryPublisher ->
        # parameter_bridge -> truth_odom_bridge -> PX4 external vision. The
        # simulated baro/GPS chain destabilizes the EKF on the dev host; the
        # EKF fuses this stream instead (EKF2_EV_CTRL/HGT_REF in the airframe).
        did = drone_id.perform(context)
        gz_topic = f'/model/x500_lidar360_{did}/odometry'
        config_dir = os.path.expanduser('~/.ros/px4_bridge_lidar_configs')
        os.makedirs(config_dir, exist_ok=True)
        config_path = os.path.join(config_dir, f'truth_bridge_{did}.yaml')
        with open(config_path, 'w') as f:
            f.write(
                f'- ros_topic_name: "{gz_topic}"\n'
                f'  gz_topic_name: "{gz_topic}"\n'
                '  ros_type_name: "nav_msgs/msg/Odometry"\n'
                '  gz_type_name: "gz.msgs.Odometry"\n'
                '  direction: GZ_TO_ROS\n'
            )
        return [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=['drone_', did, '_truth_gz_bridge'],
                output='screen',
                parameters=[{'config_file': config_path}],
                additional_env={'GZ_VERSION': 'harmonic'},
            ),
            Node(
                package='px4_bridge',
                executable='truth_odom_bridge',
                name=['drone_', did, '_truth_odom_bridge'],
                output='screen',
                parameters=[
                    {'px4_ns': px4_ns},
                    {'spawn_x': spawn_x},
                    {'spawn_y': spawn_y},
                ],
                remappings=[('truth_odom', gz_topic)],
            ),
        ]

    truth_bridge_action = OpaqueFunction(function=_make_truth_bridge_nodes)

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
    ld.add_action(spawn_x_cmd)
    ld.add_action(spawn_y_cmd)
    ld.add_action(target_system_cmd)
    ld.add_action(emergency_time_cmd)
    ld.add_action(swarm_clearance_cmd)
    ld.add_action(formation_type_cmd)
    ld.add_action(lambda_formation_cmd)
    ld.add_action(formation_num_cmd)
    ld.add_action(formation_stretch_max_cmd)
    ld.add_action(formation_nominal_leads_cmd)
    ld.add_action(max_vel_cmd)
    ld.add_action(max_acc_cmd)
    ld.add_action(virtual_ceil_height_cmd)
    ld.add_action(virtual_floor_height_cmd)
    ld.add_action(min_takeoff_height_cmd)

    ld.add_action(advanced_param_include)
    ld.add_action(traj_server_node)
    ld.add_action(odom_cloud_bridge_node)
    ld.add_action(offboard_bridge_node)
    ld.add_action(lidar_bridge_action)
    # truth_bridge_action intentionally NOT added: PX4's gz_bridge natively
    # consumes the model's odometry_with_covariance and publishes it as
    # world-frame vehicle_visual_odometry (GZBridge.cpp:244). Running our own
    # ROS-side EV forwarder alongside it double-feeds the EKF with two
    # disagreeing frames and arming is denied ("horizontal position
    # unstable", runs 58-61).

    return ld
