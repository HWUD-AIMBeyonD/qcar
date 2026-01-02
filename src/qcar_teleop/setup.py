import os
from glob import glob
from setuptools import setup

package_name = 'qcar_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='your@email.com',
    description='QCar teleop package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_qcar = qcar_teleop.cmd_vel_to_qcar:main',
            'manual_teleop = qcar_teleop.manual_teleop:main',
        ],
    },
)

