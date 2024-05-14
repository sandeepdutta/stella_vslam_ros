def generate_launch_description():
    # Launch arguments to allow command line input of config
    vocab_arg = DeclareLaunchArgument(
        'vocab_path',
        default_value='./orb_vocab.fbow',
        description='Path to the vocabulary file.'
    )
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value='./src/stella_vslam_ros/config/param.yaml',
        description='Path to the main configuration file.'
    )
    ros_params_arg = DeclareLaunchArgument(
        'ros_params_path',
        default_value='./src/stella_vslam_ros/config/ros_param.yaml',
        description='Path to the ROS parameters file.'
    )
    map_db_out_arg = DeclareLaunchArgument(
        'map_db_out_path',
        default_value='./map.msg',
        description='Output path for the map database.'
    )

    # Node configuration
    stella_vslam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_vslam',  # Optional: rename the node
        parameters=[LaunchConfiguration('ros_params_path')],
        arguments=[
            '-v', LaunchConfiguration('vocab_path'),
            '-c', LaunchConfiguration('config_path'),
            '--map-db-out', LaunchConfiguration('map_db_out_path')
        ]
    )

    return LaunchDescription([
        vocab_arg,
        config_arg,
        ros_params_arg,
        map_db_out_arg,
        stella_vslam_node
    ])
