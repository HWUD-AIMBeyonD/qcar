import os
from glob import glob
from setuptools import setup

package_name = 'qcar_nav2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
        ],
    },
)

