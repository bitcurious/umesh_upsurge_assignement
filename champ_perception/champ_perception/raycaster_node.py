#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class RaycasterNode(Node):
    def __init__(self):
        super().__init__('raycaster_node')
        
        # Parameters
        self.declare_parameter('ray_count', 16)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('angle_min', -np.pi/2)
        self.declare_parameter('angle_max', np.pi/2)
        
        # Publishers/Subscribers
        self.scan_pub = self.create_publisher(LaserScan, '/base/scan', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Simulate ray casting
        self.timer = self.create_timer(0.1, self.publish_scan)
        
    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        scan.angle_min = self.get_parameter('angle_min').value
        scan.angle_max = self.get_parameter('angle_max').value
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (
            self.get_parameter('ray_count').value - 1)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.get_parameter('min_range').value
        scan.range_max = self.get_parameter('max_range').value
        
        # Simulate distance measurements (replace with actual ray casting)
        scan.ranges = [np.random.uniform(scan.range_min, scan.range_max) 
                      for _ in range(self.get_parameter('ray_count').value)]
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = RaycasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()