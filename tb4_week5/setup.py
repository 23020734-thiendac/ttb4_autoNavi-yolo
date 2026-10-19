from glob import glob
import os

from setuptools import setup


package_name = 'tb4_week5'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'autonomous_nav_node = tb4_week5.autonomous_nav_node:main',
            'yolo_detector_node = tb4_week5.yolo_detector_node:main',
            'world_object_bbox_node = tb4_week5.world_object_bbox_node:main',
            'train_small_house_yolo = tb4_week5.train_small_house_yolo:main',
        ],
    },
)
