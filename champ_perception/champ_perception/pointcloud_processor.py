#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import numpy as np
import cv2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # Subscribers
        self.create_subscription(
            PointCloud2, 
            '/camera/depth/color/points', 
            self.pointcloud_callback, 
            10)
            
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publishers
        self.obstacle_pc_pub = self.create_publisher(
            PointCloud2, 
            '/perception/obstacle_points', 
            10)
            
        self.bridge = CvBridge()
        self.current_image = None
        
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def pointcloud_callback(self, msg):
        # Convert to numpy array
        points = point_cloud2.read_points_numpy(msg)
        
        # Simple obstacle detection (z < 0.5m)
        obstacle_mask = points[:,2] < 0.5
        obstacle_points = points[obstacle_mask]
        
        # Create new point cloud
        header = msg.header
        header.frame_id = "camera_depth_optical_frame"
        
        obstacle_pc = point_cloud2.create_cloud_xyz32(header, obstacle_points)
        self.obstacle_pc_pub.publish(obstacle_pc)
        
        # Visualize with image if available
        if self.current_image is not None:
            self.visualize_obstacles(points, obstacle_mask)
            
    def visualize_obstacles(self, points, mask):
        # Project points to image
        height, width = self.current_image.shape[:2]
        u = (points[:,0] / points[:,2] * 525 + width/2).astype(int)
        v = (points[:,1] / points[:,2] * 525 + height/2).astype(int)
        
        # Filter valid pixels
        valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v = u[valid], v[valid]
        mask = mask[valid]
        
        # Mark obstacles in red
        vis_img = self.current_image.copy()
        vis_img[v[mask], u[mask]] = [0, 0, 255]
        
        cv2.imshow("Obstacle Detection", vis_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()