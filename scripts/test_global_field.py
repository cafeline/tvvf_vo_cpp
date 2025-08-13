#!/usr/bin/env python3
"""
Global Field Generator Test Script

This script tests the TVVF-VO global field generation system by:
1. Publishing a simple map
2. Setting a goal
3. Adding dynamic obstacles
4. Monitoring the output
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Twist
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
import numpy as np
import time


class GlobalFieldTester(Node):
    def __init__(self):
        super().__init__('global_field_tester')
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/dynamic_obstacles', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.vector_field_sub = self.create_subscription(
            MarkerArray, '/tvvf_vo_vector_field', self.vector_field_callback, 10)
        
        # Test state
        self.received_cmd_vel = False
        self.received_vector_field = False
        
        # Start test sequence
        self.get_logger().info('Starting Global Field Generator test...')
        self.timer = self.create_timer(1.0, self.run_test_sequence)
        self.test_step = 0
        
    def cmd_vel_callback(self, msg):
        if not self.received_cmd_vel:
            self.get_logger().info(f'Received cmd_vel: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
            self.received_cmd_vel = True
    
    def vector_field_callback(self, msg):
        if not self.received_vector_field:
            self.get_logger().info(f'Received vector field with {len(msg.markers)} markers')
            self.received_vector_field = True
    
    def run_test_sequence(self):
        if self.test_step == 0:
            # Step 1: Publish a simple map
            self.publish_test_map()
            self.get_logger().info('Published test map (20x20)')
            self.test_step = 1
            
        elif self.test_step == 1:
            # Step 2: Set a goal
            self.publish_test_goal()
            self.get_logger().info('Published goal at (8.0, 8.0)')
            self.test_step = 2
            
        elif self.test_step == 2:
            # Step 3: Add dynamic obstacles
            self.publish_test_obstacles()
            self.get_logger().info('Published 2 dynamic obstacles')
            self.test_step = 3
            
        elif self.test_step == 3:
            # Step 4: Check results
            time.sleep(2.0)  # Wait for processing
            
            if self.received_vector_field:
                self.get_logger().info('✓ Vector field generation successful')
            else:
                self.get_logger().warn('✗ No vector field received')
            
            self.get_logger().info('Test complete!')
            self.test_step = 4
            
        elif self.test_step == 4:
            # Keep publishing obstacles to maintain the system
            self.publish_test_obstacles()
    
    def publish_test_map(self):
        """Publish a simple 20x20 occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Map metadata
        map_msg.info.resolution = 0.5
        map_msg.info.width = 20
        map_msg.info.height = 20
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Create empty map with some obstacles
        data = np.zeros((20, 20), dtype=np.int8)
        
        # Add a simple wall
        data[10, 5:15] = 100  # Horizontal wall
        
        map_msg.data = data.flatten().tolist()
        self.map_pub.publish(map_msg)
    
    def publish_test_goal(self):
        """Publish a goal point"""
        goal_msg = PointStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.point.x = 8.0
        goal_msg.point.y = 8.0
        goal_msg.point.z = 0.0
        
        self.goal_pub.publish(goal_msg)
    
    def publish_test_obstacles(self):
        """Publish dynamic obstacles"""
        marker_array = MarkerArray()
        
        # Obstacle 1
        marker1 = Marker()
        marker1.header.frame_id = 'map'
        marker1.header.stamp = self.get_clock().now().to_msg()
        marker1.id = 1
        marker1.type = Marker.CYLINDER
        marker1.action = Marker.ADD
        marker1.pose.position.x = 3.0
        marker1.pose.position.y = 3.0
        marker1.pose.position.z = 0.5
        marker1.pose.orientation.w = 1.0
        marker1.scale.x = 1.0
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.r = 1.0
        marker1.color.g = 0.0
        marker1.color.b = 0.0
        marker1.color.a = 0.8
        
        # Obstacle 2
        marker2 = Marker()
        marker2.header.frame_id = 'map'
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.id = 2
        marker2.type = Marker.CYLINDER
        marker2.action = Marker.ADD
        marker2.pose.position.x = 6.0
        marker2.pose.position.y = 6.0
        marker2.pose.position.z = 0.5
        marker2.pose.orientation.w = 1.0
        marker2.scale.x = 1.0
        marker2.scale.y = 1.0
        marker2.scale.z = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.color.a = 0.8
        
        marker_array.markers = [marker1, marker2]
        self.obstacle_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    tester = GlobalFieldTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()