import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'neo_assist'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package index registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include ALL launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Include ALL world SDF files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),

        # Include ALL URDF/xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf.xacro')),

        # Include ALL config files (YAML — used in later phases)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),

        # Include ALL RViz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),

        # Include saved maps (Phase 2+)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # maps — saved from SLAM phase
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inspiredkhalid',
    maintainer_email='inspiredkhalid@example.com',
    description='NeoAssist — Intelligent Autonomous Indoor Navigation Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # TF frame fixer — bridges Gazebo frame names to ROS2 standard
            'odom_tf_broadcaster = neo_assist.odom_tf_broadcaster:main',
            'scan_frame_fixer = neo_assist.scan_frame_fixer:main',
            'map_labeller = neo_assist.map_labeller:main',
            'navigator = neo_assist.navigator:main',
            'auto_mapper = neo_assist.auto_mapper:main',
        ],
    },
)
