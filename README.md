# umesh_upsurge_assignement
Here's a comprehensive README.md file for your CHAMP quadruped robot project:

```markdown
# CHAMP Quadruped Robot Navigation and Perception System

This repository contains the ROS 2 implementation for CHAMP quadruped robot navigation, perception, and motion control.

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your_username/champ_quadruped.git
cd champ_quadruped
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Task 1: Waypoint Follower

### Running the Waypoint Follower

1. Launch Gazebo simulation:
```bash
ros2 launch champ_config gazebo.launch.py
```

2. Launch Nav2 navigation stack:
```bash
ros2 launch champ_config navigate.launch.py rviz:=true
```

3. To publish navigation goals:
   - In RViz, use the "2D Goal Pose" tool to set waypoints
   - Or publish programmatically:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

## Task 2: Forward and Inverse Kinematics

### Forward Kinematics
Calculates foot positions from joint angles. Essential for gait planning and foot placement.

Run forward kinematics:
```bash
ros2 launch champ_kinematics fk.launch.py
```

### Inverse Kinematics
Calculates joint angles from desired foot positions. Used for precise foot placement.

Run inverse kinematics:
```bash
ros2 launch champ_kinematics ik.launch.py
```

## Task 3: Obstacle Avoidance

The perception system includes:
- Ray casting for distance measurements
- Point cloud processing for 3D obstacle detection
- Occupancy grid mapping

Run the obstacle avoidance system:
```bash
ros2 launch champ_perception perception_system.launch.py
```

### Key Components:
1. **Ray Caster**: Publishes to `/base/scan` (simulated LIDAR)
2. **Point Cloud Processor**: Processes `/camera/depth/points`
3. **Occupancy Mapper**: Creates `/map` from sensor data

## System Architecture

![System Diagram](docs/system_diagram.png)

## Troubleshooting

1. If you get TF errors:
```bash
ros2 run tf2_ros tf2_echo base_link map
```

2. To reset the simulation:
```bash
ros2 service call /reset_simulation std_srvs/srv/Empty
```

## License

Apache License 2.0
```

### Key Features of this README:

1. **Clear Installation Instructions**: Step-by-step setup guide
2. **Task-Specific Sections**: Separated by functionality
3. **Command Formatting**: Easy-to-copy code blocks
4. **Visual Structure**: Using headers and lists for readability
5. **Troubleshooting**: Common solutions for typical issues
6. **System Overview**: Brief explanation of components

To use this README:
1. Save as `README.md` in your project root
2. Update the repository URL in the clone command
3. Add your system diagram image to the `docs/` folder
4. Customize any package names or paths as needed for your specific setup

The README provides a complete starting point for users to understand and operate your CHAMP quadruped robot system.
