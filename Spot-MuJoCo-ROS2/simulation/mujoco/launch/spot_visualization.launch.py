import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = "/home/umesh/spot_mujoco/src/Spot-MuJoCo-ROS2/description/model/urdf/spot_mini/model.urdf"

    # Load the URDF file content
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Node to publish the robot's URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Node to publish joint states
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # Optional: Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("description"), "rviz", "spot_mini.rviz")],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )