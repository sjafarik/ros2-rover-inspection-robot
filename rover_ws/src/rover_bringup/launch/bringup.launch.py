from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    ros_gz_sim_path = get_package_share_directory("ros_gz_sim")
    rover_simulation_path = get_package_share_directory("rover_simulation")
    rover_description_path = get_package_share_directory("rover_description")

    simulation_world_file_path = Path(
        rover_simulation_path, "worlds", "rover_robot_world.sdf"
    ).as_posix()

    simulation_model_path = Path(
        rover_description_path, "models"
    ).as_posix()

    print(simulation_model_path)

    set_gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=simulation_model_path
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(ros_gz_sim_path, "launch", "gz_sim.launch.py").as_posix()
        ),
        launch_arguments={
            "gz_args": "-r " + simulation_world_file_path
        }.items()
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/rover_robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_pos_cmd@std_msgs/msg/Float64@ignition.msgs.Double",
            "/model/rover_robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/world/test_world/pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            "/world/test_world/model/rover_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        remappings=[
            ("/model/rover_robot/cmd_vel", "/cmd_vel"),
            ("/world/test_world/pose/info", "/world_pose_tf"),
            ("/world/test_world/model/rover_robot/joint_state", "/joint_states"),
        ],
        output="screen"
    )

    navigation_action_server = Node(
        package="rover_control",
        executable="rover_navigation_action_server",
        output="screen"
    )

    camera_capture_service_server = Node(
        package="rover_control",
        executable="rover_camera_capture_service_server",
        output="screen"
    )

    return LaunchDescription([
        set_gazebo_resource_path,
        gz_sim_launch,
        bridge,
        navigation_action_server,
        camera_capture_service_server,
    ])