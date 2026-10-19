"""Run localization and Nav2 in the AWS RoboMaker small-house world."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    package_share = get_package_share_directory('tb4_week5')
    tb4_sim_share = get_package_share_directory('tb4_sim')
    navigation_share = get_package_share_directory('turtlebot4_navigation')
    sim_launch = PathJoinSubstitution([
        package_share, 'launch', 'small_house_sim.launch.py'])
    localization_launch = PathJoinSubstitution([
        navigation_share, 'launch', 'localization.launch.py'])
    nav2_launch = PathJoinSubstitution([
        navigation_share, 'launch', 'nav2.launch.py'])
    rviz_config = PathJoinSubstitution([
        package_share, 'rviz', 'small_house_slam.rviz'])
    # Week 5 navigation is intentionally tied to the map produced in the
    # small-house world.  Do not let a legacy `map:=.../map.yaml` (maze map)
    # leak into map_server through nested launch arguments.
    small_house_map = PathJoinSubstitution([
        tb4_sim_share, 'maps', 'small_house_slam.yaml'])
    nav2_params = ReplaceString(
        source_file=PathJoinSubstitution([
            navigation_share, 'config', 'nav2.yaml']),
        replacements={
            'max_vel_x: 0.26': 'max_vel_x: 0.40',
            'max_speed_xy: 0.26': 'max_speed_xy: 0.40',
            'max_vel_theta: 1.0': 'max_vel_theta: 1.40',
            'max_velocity: [0.26, 0.0, 1.0]':
                'max_velocity: [0.40, 0.0, 1.40]',
            'min_velocity: [-0.26, 0.0, -1.0]':
                'min_velocity: [-0.40, 0.0, -1.40]',
            'max_rotational_vel: 1.0': 'max_rotational_vel: 1.40',
        })

    args = [
        DeclareLaunchArgument('ros_domain_id', default_value='68'),
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/thiendac/ros2_ws/src/TuongTacNguoi_Robot/'
                'tb4_sim/maps/small_house_slam.yaml'),
            description='SLAM map saved from the small-house world.'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('localization_delay_sec', default_value='10.0'),
        DeclareLaunchArgument('nav2_delay_sec', default_value='14.0'),
        # RViz starts after the autonomous node has had time to publish the
        # AMCL initial pose.  With fixed frame "map", opening it earlier makes
        # RobotModel red until the map -> odom transform exists.
        DeclareLaunchArgument('rviz_delay_sec', default_value='30.0'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments={
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'model': LaunchConfiguration('model'),
            # RViz is started below only after localization and Nav2 exist.
            'rviz': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    localization = TimerAction(
        period=LaunchConfiguration('localization_delay_sec'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([localization_launch]),
            launch_arguments={
                'map': small_house_map,
                'use_sim_time': 'true',
            }.items())],
    )
    navigation = TimerAction(
        period=LaunchConfiguration('nav2_delay_sec'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items())],
    )
    rviz = TimerAction(
        period=LaunchConfiguration('rviz_delay_sec'),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            condition=IfCondition(LaunchConfiguration('show_rviz')),
        )],
    )

    return LaunchDescription(
        args + [simulation, localization, navigation, rviz])
