import os
from glob import glob
from setuptools import setup

package_name = 'qcar_nav2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index + package manifest
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files (match your existing naming: *.launch.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Install config files (Nav2 + SLAM params)
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Install RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),

        # Install behavior trees (optional, but standard for Nav2)
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*')),

        # Install scripts (helper bash scripts)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='your@email.com',
    description='QCar Nav2 bringup with unified hardware interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qcar_hardware_interface = qcar_nav2_bringup.qcar_hardware_interface:main',
            'map_republisher = qcar_nav2_bringup.map_republisher:main',
            'direct_map_publisher = qcar_nav2_bringup.direct_map_publisher:main',
            'send_nav_goal = qcar_nav2_bringup.send_nav_goal:main',
            'path_visualizer = qcar_nav2_bringup.path_visualizer:main',
        ],
    },
)

