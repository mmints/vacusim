#!/usr/bin/env python3.10

'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()

    node = rclpy.create_node('hello_world_publisher')
    publisher = node.create_publisher(String, '/test', 10)

    msg = String()
    msg.data = 'Hello World'

    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Published: "%s"' % msg.data)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
