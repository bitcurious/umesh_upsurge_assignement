from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='champ_kinematics',
            executable='quadruped_ik_node',
            name='quadruped_inverse_kinematics',
            output='screen',
            parameters=[
                {'upper_leg_length': 0.141},
                {'lower_leg_length': 0.141},
                {'base_to_hip_x': 0.175},
                {'base_to_hip_y': 0.105}
            ]
        )
    ])