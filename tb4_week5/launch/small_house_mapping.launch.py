"""Run SLAM in the AWS RoboMaker small-house world."""

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


def generate_launch_description():
    package_share = get_package_share_directory('tb4_week5')
    tb4_sim_share = get_package_share_directory('tb4_sim')
    sim_launch = PathJoinSubstitution([
        package_share, 'launch', 'small_house_sim.launch.py'])
    slam_params = PathJoinSubstitution([tb4_sim_share, 'config', 'slam.yaml'])
    rviz_config = PathJoinSubstitution([
        package_share, 'rviz', 'small_house_slam.rviz'])

    args = [
        DeclareLaunchArgument('ros_domain_id', default_value='68'),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'show_rviz', default_value='true',
            description='Open the custom RViz window after Gazebo and SLAM.'),
        DeclareLaunchArgument(
            'slam_delay_sec', default_value='8.0',
            description='Wait for Gazebo, robot and sensors before SLAM.'),
        DeclareLaunchArgument(
            'rviz_delay_sec', default_value='14.0',
            description='Open RViz only after Gazebo and SLAM are running.'),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([sim_launch]),
        launch_arguments={
            'ros_domain_id': LaunchConfiguration('ros_domain_id'),
            'model': LaunchConfiguration('model'),
            # RViz is intentionally started below, after Gazebo and SLAM.
            'rviz': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    slam_node = Node(
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

    slam = TimerAction(
        period=LaunchConfiguration('slam_delay_sec'),
        actions=[slam_node],
    )

    rviz = TimerAction(
        period=LaunchConfiguration('rviz_delay_sec'),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                condition=IfCondition(LaunchConfiguration('show_rviz')),
            )
        ],
    )

    return LaunchDescription(args + [simulation, slam, rviz])
