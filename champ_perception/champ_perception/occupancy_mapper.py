#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf2_ros import Buffer, TransformListener

class OccupancyMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')
        
        # Parameters
        self.declare_parameter('map_width', 20.0)  # meters
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('resolution', 0.05)  # meters/pixel
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        
        # Subscribers
        self.create_subscription(
            LaserScan,
            '/base/scan',
            self.scan_callback,
            10)
            
        self.create_subscription(
            PointCloud2,
            '/perception/obstacle_points',
            self.pointcloud_callback,
            10)
            
        self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
            
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10)
            
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize map
        self.initialize_map()
        
    def initialize_map(self):
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        width = int(self.get_parameter('map_width').value / 
                   self.get_parameter('resolution').value)
        height = int(self.get_parameter('map_height').value / 
                    self.get_parameter('resolution').value)
        
        self.map.info.width = width
        self.map.info.height = height
        self.map.info.resolution = self.get_parameter('resolution').value
        self.map.info.origin.position.x = self.get_parameter('origin_x').value
        self.map.info.origin.position.y = self.get_parameter('origin_y').value
        self.map.data = [-1] * (width * height)  # -1 = unknown
        
    def scan_callback(self, msg):
        # Update map with laser scan data
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                msg.header.stamp)
            
            # Convert scan to map coordinates and update grid
            # (Implementation depends on your mapping algorithm)
            
            self.map.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map)
            
        except Exception as e:
            self.get_logger().error(f'TF error: {str(e)}')

    def pointcloud_callback(self, msg):
        # Update map with point cloud data
        pass
        
    def pose_callback(self, msg):
        # Update robot position in map
        pass

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()