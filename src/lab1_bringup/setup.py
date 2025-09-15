from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'lab1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[ 
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']), 
        # Include launch files 
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        # Include config files 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ], 
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    install_requires=['setuptools', 'rclpy', 'launch', 'launch_ros'],
    zip_safe=True,
    maintainer='adoolph',
    maintainer_email='adoolph@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
