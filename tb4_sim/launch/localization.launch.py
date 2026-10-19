from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_share = get_package_share_directory('turtlebot4_ignition_bringup')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    ignition_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'ignition.launch.py'
    ])
    robot_launch = PathJoinSubstitution([
        bringup_share, 'launch', 'turtlebot4_spawn.launch.py'
    ])
    localization_launch = PathJoinSubstitution([
        navigation_share, 'launch', 'localization.launch.py'
    ])

    args = [
        DeclareLaunchArgument('world', default_value='maze'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument(
            'map',
            default_value='/home/vtd/ros2_ws/src/tb4_sim/maps/map.yaml',
            description='Full path to the map YAML saved after SLAM'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ignition_launch),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
        }.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'rviz': 'true',
            'use_sim_time': 'true',
            'localization': 'false',
            'slam': 'false',
            'nav2': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription(args + [gazebo, robot, localization])
