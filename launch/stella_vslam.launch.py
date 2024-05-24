import os

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes,Node
from launch.actions import IncludeLaunchDescription,  DeclareLaunchArgument,  OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration,  PythonExpression
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition, IfCondition

def conditionalLoad(context, *args, **kwargs):
    localization    = LaunchConfiguration('localization').perform(context)
    vocab_path      = LaunchConfiguration('vocab_path').perform(context)
    config_path     = LaunchConfiguration('config_path').perform(context)
    map_db_path     = LaunchConfiguration('map_db_path').perform(context)
    container_name = LaunchConfiguration('component_container_name').perform(context)
    actions = []
    # mappng launch
    if localization == 'False' or localization == 'false':
        actions.append(LoadComposableNodes (target_container=container_name,
                                            composable_node_descriptions=[ComposableNode(
                                            package='stella_vslam_ros',
                                            plugin='stella_vslam_ros::System',
                                            namespace='stella_vslam',
                                            parameters=[{'publish_tf' : True,
                                                         'use_exact_time' : True,
                                                         'odom2d' : False ,
                                                         'vocab_file_path' : vocab_path,
                                                         'setting_file_path' : config_path,
                                                         'log_level' : 'info',
                                                         'map_db_path_in': map_db_path,
                                                         'map_db_path_out': map_db_path,
                                                         'viewer': "pointcloud_publisher",
                                                         "camera_frame": "rs_bottom_color_optical_frame",
                                                         'disable_mapping': False,
                                                         'temporal_mapping': False,
                                                         'reset_on_start': False
                                                         }],
                                            extra_arguments=[{'use_intra_process_comms': True}],
                                            remappings=[
                                                ('/stella_vslam/camera/left/image_raw',  '/rs_bottom/camera/infra1/image_rect_raw'),
                                                ('/stella_vslam/camera/right/image_raw', '/rs_bottom/camera/infra2/image_rect_raw'),
                                                ('/stella_vslam/run_slam/wheel_odom', '/diffbot_base_controller/odom'),
                                            ])]))
    else:
        actions.append(LoadComposableNodes (target_container=container_name,
                                     composable_node_descriptions=[ComposableNode(
                                     package='stella_vslam_ros',
                                     plugin='stella_vslam_ros::System',
                                     namespace='stella_vslam',
                                     parameters=[{'publish_tf' : True,
                                                  'use_exact_time' : True,
                                                  'odom2d' : False ,
                                                  'vocab_file_path' : vocab_path,
                                                  'setting_file_path' : config_path,
                                                  'log_level' : 'info',
                                                  'map_db_path_in': map_db_path,
                                                  'map_db_path_out': map_db_path,
                                                  'viewer': "pointcloud_publisher",
                                                  "camera_frame": "rs_bottom_color_optical_frame",
                                                  'disable_mapping': True,
                                                  'temporal_mapping': True
                                                  }],
                                     extra_arguments=[{'use_intra_process_comms': True}],
                                     remappings=[
                                                ('/stella_vslam/camera/left/image_raw',  '/rs_bottom/camera/infra1/image_rect_raw'),
                                                ('/stella_vslam/camera/right/image_raw', '/rs_bottom/camera/infra2/image_rect_raw'),
                                                ('/stella_vslam/run_slam/wheel_odom', '/diffbot_base_controller/odom'),
                                     ])]))
    return actions

def generate_launch_description():
    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = DeclareLaunchArgument('attach_to_shared_component_container', default_value='False')
    attach_to_shared_component_container     = LaunchConfiguration('attach_to_shared_component_container')
    component_container_name_arg = DeclareLaunchArgument('component_container_name', default_value='stella_vslam_container')
    component_container_name     = LaunchConfiguration('component_container_name', default='stella_vslam_container')

    stella_vslam_dir = get_package_share_directory('stella_vslam_ros')
    # Launch arguments to allow command line input of config
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value= 'False',
        description='Run in localtion mode.'
    )
    localization = LaunchConfiguration('localization')
    vocab_arg = DeclareLaunchArgument(
        'vocab_path',
        default_value= os.path.join(stella_vslam_dir,'config','orb_vocab.fbow'),
        description='Path to the vocabulary file.'
    )
    vocab_path = LaunchConfiguration('vocab_path')
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(stella_vslam_dir,'config','param.yaml'),
        description='Path to the main configuration file.'
    )
    config_path =  LaunchConfiguration('config_path')
    map_db_arg = DeclareLaunchArgument(
        'map_db_path',
        default_value='./map.msg',
        description='Output path for the map database.'
    )
    map_db_path = LaunchConfiguration('map_db_path')
    # start the viewer
    socket_viewer = ExecuteProcess(
        cmd=['node','/workspaces/socket_viewer/app.js'],
        cwd='/workspaces/socket_viewer',
        output='both')
    
    # If we do not attach to a shared component container we have to create our own container.
    stella_vslam_container = Node(
        name=component_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='both',
        condition=UnlessCondition(attach_to_shared_component_container)
    )

    return LaunchDescription([
        attach_to_shared_component_container_arg,
        component_container_name_arg,
        socket_viewer,
        localization_arg,
        vocab_arg,
        config_arg,
        map_db_arg,
        stella_vslam_container,
        OpaqueFunction(function=conditionalLoad)
    ])
