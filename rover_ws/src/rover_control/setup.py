from setuptools import find_packages, setup

package_name = 'rover_control'

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
    maintainer='Saeed Jafari Kang',
    maintainer_email='sjafarik@mtu.edu',
    description='ROS2 control nodes for rover navigation and camera capture.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rover_navigation_action_server = rover_control.rover_navigation_action_server:main',
            'rover_navigation_action_client = rover_control.rover_navigation_action_client:main',
            'rover_camera_capture_service_server = rover_control.rover_camera_capture_service_server:main',
            'rover_camera_capture_service_client = rover_control.rover_camera_capture_service_client:main',
            'obstacle_avoidance = rover_control.obstacle_avoidance:main',
        ],
    },
)