#!/usr/bin/env python3

"""
Minimal long-running node with enable_logger_service=True, for exercising
swri_console's node-list "Set Log Level" context menu entry on Iron and newer.

Usage: python3 logger_service_test_node.py
"""

import rclpy
from rclpy.node import Node


class LoggerServiceTestNode(Node):
    def __init__(self):
        super().__init__('logger_service_test_node', enable_logger_service=True)
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.get_logger().info('still alive')


def main():
    rclpy.init()
    rclpy.spin(LoggerServiceTestNode())


if __name__ == '__main__':
    main()
