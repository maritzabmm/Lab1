from setuptools import find_packages, setup

package_name = 'task3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'custom_interface'],
    zip_safe=True,
    maintainer='algoritmic12',
    maintainer_email='algoritmic12@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = task3.calc_dist_server:main',
            'client = task3.calc_dist_client:main',
        ],
    },
)
