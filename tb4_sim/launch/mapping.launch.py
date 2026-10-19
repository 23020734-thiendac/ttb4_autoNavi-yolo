from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('turtlebot4_ignition_bringup')
    project_share = get_package_share_directory('tb4_sim')
    bringup_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'turtlebot4_ignition.launch.py'
    ])
    slam_params = PathJoinSubstitution([project_share, 'config', 'slam.yaml'])
    args = [
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'slam': 'false',
            'nav2': 'false',
            'localization': 'false',
            'rviz': 'true',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata'),
        ],
    )

    return LaunchDescription(args + [simulation, slam])
