from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('turtlebot4_ignition_bringup')
    bringup_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'turtlebot4_ignition.launch.py'
    ])
    args = [
        DeclareLaunchArgument('world', default_value='maze',
                              description='TurtleBot 4 world: depot, maze, warehouse'),
        DeclareLaunchArgument('model', default_value='standard',
                              description='TurtleBot 4 model: standard or lite'),
        DeclareLaunchArgument('rviz', default_value='true',
                              choices=['true', 'false']),
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
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # The stock TurtleBot 4 RViz config uses `map` as its Fixed Frame. In this
    # simulation-only launch there is no SLAM/localization node to publish
    # map->odom, so provide an identity link for visualization. Mapping and
    # navigation launch files do not use this placeholder transform.
    rviz_frame_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='part1_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [simulation, rviz_frame_link])
