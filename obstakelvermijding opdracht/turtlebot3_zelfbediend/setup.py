from setuptools import find_packages, setup
from setuptools import setup
import os
from glob import glob
package_name = 'turtlebot3_zelfbediend'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boross',
    maintainer_email='boross@todo.todo',
    description='Zelfbediende turtlebot3 in ros2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zelfbediend_nav = turtlebot3_zelfbediend.zelfbediend_nav:main'
        ],
    },
)
