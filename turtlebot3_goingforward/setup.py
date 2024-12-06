from setuptools import setup

package_name = 'turtlebot3_goingforward'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Package manifest
        ('share/' + package_name + '/launch', ['launch/forward_launch.py']),  # Launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple node to move a TurtleBot3 forward until an obstacle is detected.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward = turtlebot3_goingforward.forward:main',  # Executable definition
        ],
    },
)
