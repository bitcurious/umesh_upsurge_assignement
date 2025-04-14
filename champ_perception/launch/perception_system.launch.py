from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='champ_perception',
            executable='raycaster_node',
            name='raycaster_node',
            output='screen',
            parameters=[{
                'ray_count': 32,
                'max_range': 5.0,
                'min_range': 0.1,
                'angle_min': -1.57,  # -π/2
                'angle_max': 1.57    # π/2
            }]
        ),
        Node(
            package='champ_perception',
            executable='pointcloud_processor',
            name='pointcloud_processor',
            output='screen'
        ),
        Node(
            package='champ_perception',
            executable='occupancy_mapper',
            name='occupancy_mapper',
            output='screen',
            parameters=[{
                'map_width': 20.0,
                'map_height': 20.0,
                'resolution': 0.05,
                'origin_x': -10.0,
                'origin_y': -10.0
            }]
        )
    ])