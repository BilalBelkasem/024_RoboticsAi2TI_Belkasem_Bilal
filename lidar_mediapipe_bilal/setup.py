from setuptools import setup
import os
from glob import glob

package_name = 'lidar_mediapipe_bilal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'mediapipe',
        'numpy'
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for controlling robot with gestures and LIDAR',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_detection = lidar_mediapipe_bilal.gesture_detection:main',
            'obstacle_avoidance = lidar_mediapipe_bilal.obstacle_avoidance:main',
        ],
    },
)
