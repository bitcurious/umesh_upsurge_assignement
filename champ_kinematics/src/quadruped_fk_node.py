#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class QuadrupedForwardKinematics(Node):
    def __init__(self):
        super().__init__('quadruped_forward_kinematics')
    
        self.base_to_hip_x = 0.175  # meters
        self.base_to_hip_y = 0.105  # meters
        self.upper_leg_length = 0.141  # meters (hip_to_upper_leg_distance + upper_leg_to_lower_leg_distance)
        self.lower_leg_length = 0.141  # meters (lower_leg_to_foot_distance)
        
        self.leg_prefixes = ['lf', 'rf', 'lh', 'rh']
        
        # DH Parameters for each leg joint (hip, thigh, calf)
        self.dh_params = [
            {'a': 0.0, 'd': 0.0, 'alpha': np.pi/2},    # Hip joint (abduction/adduction)
            {'a': self.upper_leg_length, 'd': 0.0, 'alpha': 0.0},  # Thigh joint
            {'a': self.lower_leg_length, 'd': 0.0, 'alpha': 0.0}   # Calf joint
        ]
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
            
        # Publishers
        self.foot_pose_pub = self.create_publisher(
            PoseArray, '/foot_poses', 10)
            
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Quadruped Forward Kinematics Node Initialized with CHAMP parameters")

    def dh_matrix(self, theta, a, d, alpha):
        """Create Denavit-Hartenberg transformation matrix"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def get_hip_offset(self, leg_prefix):
        """Get hip offset from base_link based on leg position"""
        x_sign = 1.0 if leg_prefix in ['lf', 'lh'] else -1.0  # Left legs have positive X
        y_sign = 1.0 if leg_prefix in ['lf', 'rf'] else -1.0  # Front legs have positive Y
        
        return (
            x_sign * self.base_to_hip_x,
            y_sign * self.base_to_hip_y,
            0.0  # Z offset is 0 in your configuration
        )

    def calculate_fk(self, joint_angles, leg_prefix):
        """Calculate forward kinematics for one leg"""
        # Get hip offset from base
        hip_x, hip_y, hip_z = self.get_hip_offset(leg_prefix)
        
        # Base to hip transform
        T_base_hip = np.eye(4)
        T_base_hip[0, 3] = hip_x
        T_base_hip[1, 3] = hip_y
        T_base_hip[2, 3] = hip_z
        
        # Chain DH transforms for the leg
        T_hip_foot = np.eye(4)
        for i in range(3):
            theta = joint_angles[i]
            dh = self.dh_params[i]
            T_i = self.dh_matrix(theta, dh['a'], dh['d'], dh['alpha'])
            T_hip_foot = T_hip_foot @ T_i
        
        # Final transform from base to foot
        T_base_foot = T_base_hip @ T_hip_foot
        
        # Extract position
        position = T_base_foot[:3, 3]
        
        # Publish TF
        self.publish_tf(T_base_foot, 'base_link', f'{leg_prefix}_foot')
        
        return position

    def publish_tf(self, transform, parent_frame, child_frame):
        """Publish transform to TF"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        # Position
        t.transform.translation.x = transform[0, 3]
        t.transform.translation.y = transform[1, 3]
        t.transform.translation.z = transform[2, 3]
        
        # Rotation (convert rotation matrix to quaternion)
        # For a more robust implementation, consider using tf_transformations
        trace = transform[0, 0] + transform[1, 1] + transform[2, 2]
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            t.transform.rotation.w = 0.25 * S
            t.transform.rotation.x = (transform[2, 1] - transform[1, 2]) / S
            t.transform.rotation.y = (transform[0, 2] - transform[2, 0]) / S
            t.transform.rotation.z = (transform[1, 0] - transform[0, 1]) / S
        elif (transform[0, 0] > transform[1, 1]) and (transform[0, 0] > transform[2, 2]):
            S = np.sqrt(1.0 + transform[0, 0] - transform[1, 1] - transform[2, 2]) * 2
            t.transform.rotation.w = (transform[2, 1] - transform[1, 2]) / S
            t.transform.rotation.x = 0.25 * S
            t.transform.rotation.y = (transform[0, 1] + transform[1, 0]) / S
            t.transform.rotation.z = (transform[0, 2] + transform[2, 0]) / S
        elif transform[1, 1] > transform[2, 2]:
            S = np.sqrt(1.0 + transform[1, 1] - transform[0, 0] - transform[2, 2]) * 2
            t.transform.rotation.w = (transform[0, 2] - transform[2, 0]) / S
            t.transform.rotation.x = (transform[0, 1] + transform[1, 0]) / S
            t.transform.rotation.y = 0.25 * S
            t.transform.rotation.z = (transform[1, 2] + transform[2, 1]) / S
        else:
            S = np.sqrt(1.0 + transform[2, 2] - transform[0, 0] - transform[1, 1]) * 2
            t.transform.rotation.w = (transform[1, 0] - transform[0, 1]) / S
            t.transform.rotation.x = (transform[0, 2] + transform[2, 0]) / S
            t.transform.rotation.y = (transform[1, 2] + transform[2, 1]) / S
            t.transform.rotation.z = 0.25 * S
        
        self.tf_broadcaster.sendTransform(t)

    def joint_callback(self, msg):
        """Process joint states and calculate FK"""
        if len(msg.position) < 12:  # 3 joints per leg * 4 legs
            self.get_logger().warn("Not enough joint positions received")
            return
            
        foot_poses = PoseArray()
        foot_poses.header = msg.header
        foot_poses.header.frame_id = "base_link"
        
        # Process each leg
        for i, leg_prefix in enumerate(self.leg_prefixes):
            # Get joint angles for this leg (assuming order matches XACRO: lf_hip, lf_thigh, lf_calf, rf_hip, ...)
            joint_angles = msg.position[i*3 : i*3+3]
            
            # Calculate FK
            foot_position = self.calculate_fk(joint_angles, leg_prefix)
            
            # Create Pose message
            pose = Pose()
            pose.position.x = foot_position[0]
            pose.position.y = foot_position[1]
            pose.position.z = foot_position[2]
            foot_poses.poses.append(pose)
        
        self.foot_pose_pub.publish(foot_poses)

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedForwardKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()