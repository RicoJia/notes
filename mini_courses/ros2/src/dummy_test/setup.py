from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dummy_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='ricojia',
    maintainer_email='ricojia@todo.todo',
    description='A ROS 2 package for testing.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_test = dummy_test.demo:main',
        ],
    },
)

