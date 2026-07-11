from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    go2_core_dir = get_package_share_directory('go2_core')
    go2_navigation_dir = get_package_share_directory('go2_navigation')
    go2_perception_dir = get_package_share_directory('go2_perception')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_config = os.path.join(
        go2_navigation_dir,
        'config',
        'nav2_params.yaml'
    )
    rviz_config_path = os.path.join(
        go2_core_dir,
        'config',
        'default.rviz'
    )
    default_map_path = os.path.join(
        os.path.expanduser('~'),
        'go2_maps',
        'go2_map.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map yaml file to load'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    video_enable_arg = DeclareLaunchArgument(
        'video_enable',
        default_value='false',
        description='Whether to start the camera video stream node'
    )
    auto_localize_arg = DeclareLaunchArgument(
        'auto_localize',
        default_value='false',
        description='Run AMCL global localization and spin in place after startup'
    )
    auto_localize_start_delay_arg = DeclareLaunchArgument(
        'auto_localize_start_delay',
        default_value='8.0',
        description='Delay before starting auto global localization'
    )
    auto_localize_spin_duration_arg = DeclareLaunchArgument(
        'auto_localize_spin_duration',
        default_value='18.0',
        description='How long to rotate during auto global localization'
    )
    auto_localize_angular_speed_arg = DeclareLaunchArgument(
        'auto_localize_angular_speed',
        default_value='0.2',
        description='Angular speed used during auto global localization'
    )
    navigation_start_delay_arg = DeclareLaunchArgument(
        'navigation_start_delay',
        default_value='6.0',
        description='Delay before starting Nav2 navigation after localization starts'
    )

    go2_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_core_dir, 'launch', 'go2_base.launch.py')
        ),
        launch_arguments={
            'video_enable': LaunchConfiguration('video_enable'),
            'image_topic': '/camera/image_raw',
            'tcp_enable': 'false',
            'tcp_host': '127.0.0.1',
            'tcp_port': '5432',
            'target_fps': '30',
        }.items()
    )

    pointcloud_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_perception_dir, 'launch', 'go2_pointcloud_process.launch.py')
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_config,
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_config,
            'autostart': LaunchConfiguration('autostart'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    auto_global_localizer_node = Node(
        package='go2_navigation',
        executable='auto_global_localizer.py',
        name='auto_global_localizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_localize')),
        parameters=[{
            'start_delay': LaunchConfiguration('auto_localize_start_delay'),
            'spin_duration': LaunchConfiguration('auto_localize_spin_duration'),
            'angular_speed': LaunchConfiguration('auto_localize_angular_speed'),
            'cmd_vel_topic': '/cmd_vel',
            'global_localization_service': '/reinitialize_global_localization',
        }]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        autostart_arg,
        use_rviz_arg,
        video_enable_arg,
        auto_localize_arg,
        auto_localize_start_delay_arg,
        auto_localize_spin_duration_arg,
        auto_localize_angular_speed_arg,
        navigation_start_delay_arg,
        go2_base_launch,
        pointcloud_process_launch,
        localization_launch,
        TimerAction(
            period=LaunchConfiguration('navigation_start_delay'),
            actions=[navigation_launch]
        ),
        rviz_node,
        auto_global_localizer_node,
    ])
