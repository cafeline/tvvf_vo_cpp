#!/usr/bin/env python3
"""
Test script to verify cmd_vel output

This script:
1. Publishes a simple map
2. Publishes a clicked_point (goal)
3. Monitors /cmd_vel output
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Header
import numpy as np


class CmdVelTester(Node):
    def __init__(self):
        super().__init__('cmd_vel_tester')
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # State
        self.cmd_vel_received = False
        self.cmd_vel_count = 0
        
        # Start test
        self.get_logger().info('Starting cmd_vel test...')
        self.timer = self.create_timer(1.0, self.run_test)
        self.test_step = 0
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel_count += 1
        if not self.cmd_vel_received:
            self.get_logger().info(f'✓ cmd_vel received! linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
            self.cmd_vel_received = True
        else:
            # 継続的な出力を確認
            if self.cmd_vel_count % 10 == 0:
                self.get_logger().info(f'cmd_vel #{self.cmd_vel_count}: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
    
    def run_test(self):
        if self.test_step == 0:
            # Step 1: Publish map
            self.publish_simple_map()
            self.get_logger().info('Published map')
            self.test_step = 1
            
        elif self.test_step == 1:
            # Step 2: Wait a bit for map processing
            self.test_step = 2
            
        elif self.test_step == 2:
            # Step 3: Publish goal
            self.publish_goal()
            self.get_logger().info('Published goal at (5.0, 5.0)')
            self.test_step = 3
            
        elif self.test_step == 3:
            # Step 4: Check results
            if self.cmd_vel_received:
                self.get_logger().info(f'✓ Test PASSED! Received {self.cmd_vel_count} cmd_vel messages')
            else:
                self.get_logger().warn('⚠ No cmd_vel received yet...')
            
            # Continue publishing goal to maintain navigation
            if self.cmd_vel_count < 100:  # Stop after 100 messages
                self.publish_goal()
    
    def publish_simple_map(self):
        """Publish a simple empty map"""
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
        
        # Empty map (all free space)
        data = np.zeros((20, 20), dtype=np.int8)
        map_msg.data = data.flatten().tolist()
        
        self.map_pub.publish(map_msg)
    
    def publish_goal(self):
        """Publish a goal point"""
        goal_msg = PointStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.point.x = 5.0
        goal_msg.point.y = 5.0
        goal_msg.point.z = 0.0
        
        self.goal_pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    
    tester = CmdVelTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        if tester.cmd_vel_received:
            tester.get_logger().info('✓ Test completed successfully!')
        else:
            tester.get_logger().error('✗ Test failed - no cmd_vel received')
        
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()