from glob import glob
import os

from setuptools import setup


package_name = 'tb4_sim'

# Install a small overlay of irobot_create_description. The upstream Create 3
# description selects Ogre for the Sensors system, while this TurtleBot 4
# simulation uses a GPU LiDAR that needs Ogre2 in Gazebo Fortress.
description_data_files = []
for package_to_overlay in ('irobot_create_description', 'turtlebot4_description'):
    source_root = os.path.join('vendor', package_to_overlay)
    for source_file in glob(os.path.join(source_root, '**', '*'), recursive=True):
        if os.path.isfile(source_file):
            relative_path = os.path.relpath(source_file, source_root)
            install_dir = os.path.join(
                'share', package_to_overlay, os.path.dirname(relative_path)
            )
            description_data_files.append((install_dir, [source_file]))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
         ['resource/irobot_create_description']),
        ('share/ament_index/resource_index/packages',
         ['resource/turtlebot4_description']),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ] + description_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TurtleBot 4 simulation student',
    maintainer_email='student@example.com',
    description='Launch wrappers and mapping workspace for TurtleBot 4 simulation.',
    license='Apache-2.0',
)
