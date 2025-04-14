#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class QuadrupedInverseKinematics(Node):
    def __init__(self):
        super().__init__('quadruped_inverse_kinematics')
        
        self.upper_leg_length = 0.141  # Thigh length (L2)
        self.lower_leg_length = 0.141  # Calf length (L3)
        self.base_to_hip_x = 0.175
        self.base_to_hip_y = 0.105
        
        # Leg configuration
        self.leg_prefixes = ['lf', 'rf', 'lh', 'rh']  # left-front, right-front, left-hind, right-hind
        
        # Subscribers
        self.foot_target_sub = self.create_subscription(
            PoseArray, '/foot_target_poses', self.foot_target_callback, 10)
            
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, '/joint_group_effort_controller/joint_trajectory', 10)
        
        # TF listener for base frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Quadruped Inverse Kinematics Node Initialized")

    def get_hip_offset(self, leg_prefix):
        """Get hip offset from base_link based on leg position"""
        x_sign = 1.0 if leg_prefix in ['lf', 'lh'] else -1.0  # Left legs have positive X
        y_sign = 1.0 if leg_prefix in ['lf', 'rf'] else -1.0  # Front legs have positive Y
        
        return (
            x_sign * self.base_to_hip_x,
            y_sign * self.base_to_hip_y,
            0.0  # Z offset
        )

    def calculate_ik(self, x, y, z, leg_prefix):
        """
        Calculate inverse kinematics for one leg
        Returns: [θ1 (hip), θ2 (thigh), θ3 (calf)] in radians
        """
        # Get hip offset and transform to hip frame
        hip_x, hip_y, _ = self.get_hip_offset(leg_prefix)
        x_hip = x - hip_x
        y_hip = y - hip_y
        z_hip = z
        
        # θ1 - Hip rotation (abduction/adduction)
        θ1 = np.arctan2(y_hip, x_hip)
        
        # Project to leg plane
        r = np.sqrt(x_hip**2 + y_hip**2)
        z = z_hip
        
        # θ3 - Calf angle (using law of cosines)
        L2 = self.upper_leg_length
        L3 = self.lower_leg_length
        D = (r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)
        D = np.clip(D, -1.0, 1.0)  # Ensure valid domain for arccos
        
        θ3 = np.arccos(D)
        
        # θ2 - Thigh angle
        θ2 = np.arctan2(z, r) - np.arctan2(L3 * np.sin(θ3), L2 + L3 * np.cos(θ3))
        
        return [θ1, θ2, θ3]

    def foot_target_callback(self, msg):
        """Process desired foot positions and calculate joint angles"""
        if len(msg.poses) != 4:
            self.get_logger().warn("Received {} foot targets, expected 4".format(len(msg.poses)))
            return
            
        # Create joint trajectory message
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = [
            'lf_hip_joint', 'lf_thigh_joint', 'lf_calf_joint',
            'rf_hip_joint', 'rf_thigh_joint', 'rf_calf_joint',
            'lh_hip_joint', 'lh_thigh_joint', 'lh_calf_joint',
            'rh_hip_joint', 'rh_thigh_joint', 'rh_calf_joint'
        ]
        
        point = JointTrajectoryPoint()
        
        try:
            # Transform foot targets to base frame if necessary
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time())
            
            # Calculate IK for each leg
            for i, leg_prefix in enumerate(self.leg_prefixes):
                pose = msg.poses[i]
                
                # Transform point to base frame if needed
                if msg.header.frame_id != 'base_link':
                    # Apply transform (simplified - for full implementation use tf2_geometry_msgs)
                    x = pose.position.x + transform.transform.translation.x
                    y = pose.position.y + transform.transform.translation.y
                    z = pose.position.z + transform.transform.translation.z
                else:
                    x, y, z = pose.position.x, pose.position.y, pose.position.z
                
                # Calculate IK
                joint_angles = self.calculate_ik(x, y, z, leg_prefix)
                point.positions.extend(joint_angles)
            
            # Set trajectory point timing
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 50000000  # 50ms
            
            traj_msg.points.append(point)
            self.joint_cmd_pub.publish(traj_msg)
            
        except Exception as e:
            self.get_logger().error('TF lookup failed: {}'.format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedInverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()