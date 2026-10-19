"""Launch TurtleBot 4 localization, Nav2 navigation and world-object boxes."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('tb4_week5')
    navigation_launch = PathJoinSubstitution([
        package_share, 'launch', 'small_house_navigation.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='68',
            description='ROS 2 domain ID dùng cho toàn bộ phiên mô phỏng.'),
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value=LaunchConfiguration('ros_domain_id')),
        DeclareLaunchArgument(
            'world', default_value='warehouse',
            description=(
                'Compatibility argument. The small-house SDF internal world '
                'name and sensor bridges use warehouse.')),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map',
            default_value=(
                '/home/thiendac/ros2_ws/src/TuongTacNguoi_Robot/'
                'tb4_sim/maps/small_house_slam.yaml')),
        DeclareLaunchArgument(
            'image_topic', default_value='/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/oakd/rgb/preview/camera_info'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/oakd/rgb/preview/depth'),
        DeclareLaunchArgument(
            'world_file',
            default_value=(
                '/home/thiendac/ros2_ws/src/TuongTacNguoi_Robot/'
                'aws-robomaker-small-house-world/worlds/small_house.world'),
            description='SDF world dung de lay ten va pose vat the.'),
        DeclareLaunchArgument('world_frame', default_value='map'),
        DeclareLaunchArgument(
            'max_bbox_area_ratio',
            default_value='0.60',
            description='Bo qua box chiem qua nhieu dien tich anh.'),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=PathJoinSubstitution([
                package_share, 'models', 'small_house_best.pt']),
            description='Tham so cu, detector world-object khong su dung.'),
        DeclareLaunchArgument('goal_count', default_value='3'),
        DeclareLaunchArgument(
            'auto_undock',
            default_value='true',
            description='Goi action /undock truoc khi gui Nav2 goal.'),
        DeclareLaunchArgument(
            'undock_timeout_sec',
            default_value='45.0',
            description='Thoi gian toi da cho robot roi dock.'),
        DeclareLaunchArgument('run_navigation', default_value='true'),
        DeclareLaunchArgument(
            'run_yolo',
            default_value='true',
            description=(
                'Giữ tên tương thích với lệnh cũ; true sẽ chạy detector '
                'world-object, không chạy YOLO.')),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument(
            'yolo_confidence', default_value='0.25',
            description='Tham so cu, detector world-object khong su dung.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Start automatic localization before RViz (RViz opens at 30 s).
        # The node still waits for AMCL/Nav2 before moving the robot.
        DeclareLaunchArgument('autonomy_delay_sec', default_value='17.0'),
        DeclareLaunchArgument('yolo_delay_sec', default_value='12.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'model': LaunchConfiguration('model'),
                'map': LaunchConfiguration('map'),
                'show_rviz': LaunchConfiguration('rviz'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
            }.items()),
        TimerAction(
            period=LaunchConfiguration('autonomy_delay_sec'),
            actions=[Node(
                package='tb4_week5',
                executable='autonomous_nav_node',
                name='week5_autonomous_nav',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        package_share, 'config', 'goals.yaml']),
                    {
                        # The initial-pose stamp must use the same clock as
                        # Gazebo, AMCL and Nav2.  A wall-clock stamp here makes
                        # AMCL reject/lose the pose and map -> odom never
                        # becomes available to RViz.
                        'use_sim_time': True,
                        'initial_x': LaunchConfiguration('x'),
                        'initial_y': LaunchConfiguration('y'),
                        'initial_yaw': LaunchConfiguration('yaw'),
                        'goal_count': LaunchConfiguration('goal_count'),
                        'auto_undock': LaunchConfiguration('auto_undock'),
                        'undock_timeout_sec': LaunchConfiguration(
                            'undock_timeout_sec'),
                    },
                ],
                condition=IfCondition(
                    LaunchConfiguration('run_navigation')))]),
        TimerAction(
            period=LaunchConfiguration('yolo_delay_sec'),
            actions=[Node(
                package='tb4_week5',
                executable='world_object_bbox_node',
                name='week5_world_object_bbox',
                output='screen',
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'camera_info_topic': LaunchConfiguration(
                        'camera_info_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'world_file': LaunchConfiguration('world_file'),
                    'world_frame': LaunchConfiguration('world_frame'),
                    'max_bbox_area_ratio': ParameterValue(
                        LaunchConfiguration('max_bbox_area_ratio'),
                        value_type=float),
                    'show_window': LaunchConfiguration('show_window'),
                }],
                condition=IfCondition(LaunchConfiguration('run_yolo')))]),
    ])
