import os
from glob import glob
from setuptools import setup

package_name = 'qcar_http_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moix-07',
    maintainer_email='moizsaeed2004@gmail.com',
    description='Polls an HTTP pose endpoint and publishes nav_msgs/Odometry on /odom_opti',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'http_odom_node = qcar_http_odom.http_odom_node:main',
        ],
    },
)
