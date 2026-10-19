"""Launch Gazebo Sim with the AWS RoboMaker small-house world."""

import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _default_paths():
    # With --symlink-install this resolves to the source repository. Keep the
    # explicit fallback for a normal non-symlink install.
    repository = Path(__file__).resolve().parents[2]
    world_root = repository / 'aws-robomaker-small-house-world'
    if not world_root.is_dir():
        world_root = Path(
            '/home/thiendac/ros2_ws/src/TuongTacNguoi_Robot/'
            'aws-robomaker-small-house-world')
    return world_root


def _nvidia_driver_is_usable():
    """Return true only when the NVIDIA runtime is actually responding."""
    nvidia_smi = shutil.which('nvidia-smi')
    if not nvidia_smi:
        return False
    try:
        subprocess.run(
            [nvidia_smi, '--query-gpu=name', '--format=csv,noheader'],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return True


def generate_launch_description():
    world_root = _default_paths()
    bringup_share = get_package_share_directory('turtlebot4_ignition_bringup')
    create_ignition_share = get_package_share_directory(
        'irobot_create_ignition_bringup')
    turtlebot_description_share = get_package_share_directory(
        'turtlebot4_description')
    create_description_share = get_package_share_directory(
        'irobot_create_description')
    gui_plugins_share = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    create_plugins_share = get_package_share_directory(
        'irobot_create_ignition_plugins')
    ros_ign_gazebo_share = get_package_share_directory('ros_ign_gazebo')

    # A stale /proc/driver/nvidia entry is not enough to use NVIDIA. If the
    # driver is not responding, force llvmpipe so Gazebo does not open a black
    # window and hang while creating the OGRE2 scene.
    if _nvidia_driver_is_usable():
        render_environment = [
            SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
            SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        ]
    else:
        render_environment = [
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
            SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe'),
        ]
    render_environment.append(
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'))

    world_file = LaunchConfiguration('world_file')
    models_path = LaunchConfiguration('models_path')
    world_path = LaunchConfiguration('world_path')
    model = LaunchConfiguration('model')

    resource_paths = [
        models_path,
        os.pathsep,
        world_path,
        os.pathsep,
        str(world_root),
        os.pathsep,
        os.path.join(bringup_share, 'worlds'),
        os.pathsep,
        os.path.join(create_ignition_share, 'worlds'),
        os.pathsep,
        str(Path(turtlebot_description_share).parent.resolve()),
        os.pathsep,
        str(Path(create_description_share).parent.resolve()),
    ]
    gui_plugin_paths = [
        os.path.join(gui_plugins_share, 'lib'),
        os.pathsep,
        os.path.join(create_plugins_share, 'lib'),
    ]

    ignition_launch = PathJoinSubstitution([
        ros_ign_gazebo_share, 'launch', 'ign_gazebo.launch.py'])
    gui_config = PathJoinSubstitution([
        bringup_share, 'gui', model, 'gui.config'])
    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments={
            'gz_args': [
                world_file,
                ' -r -v 4 --gui-config ',
                gui_config,
            ],
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_share, 'launch', 'turtlebot4_spawn.launch.py'])
        ]),
        launch_arguments={
            'model': model,
            # This is the internal <world name=...> from small_house.world.
            # It must be passed explicitly; otherwise an outer `world:=maze`
            # leaks into the TurtleBot sensor bridge and /scan stays empty.
            'world': LaunchConfiguration('bridge_world_name'),
            'rviz': LaunchConfiguration('rviz'),
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

    return LaunchDescription(render_environment + [
        DeclareLaunchArgument(
            'ros_domain_id', default_value='68',
            description='ROS 2 domain ID dùng cho small_house.'),
        DeclareLaunchArgument(
            'world_file',
            default_value=str(world_root / 'worlds' / 'small_house.world'),
            description='AWS RoboMaker small-house SDF world.'),
        DeclareLaunchArgument(
            'models_path',
            default_value=str(world_root / 'models'),
            description='Directory containing the small-house model:// assets.'),
        DeclareLaunchArgument(
            'world_path',
            default_value=str(world_root / 'worlds'),
            description='Directory containing the small-house world file.'),
        DeclareLaunchArgument(
            'bridge_world_name',
            default_value='warehouse',
            description=(
                'Internal SDF world name used by lidar/camera bridges.')),
        DeclareLaunchArgument('model', default_value='standard'),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugin_paths),
        ignition,
        clock_bridge,
        robot_spawn,
    ])
