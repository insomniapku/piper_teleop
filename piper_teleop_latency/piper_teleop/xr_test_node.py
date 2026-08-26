#!/usr/bin/env python3
"""
Standalone XR Interface Test Node
Tests XR data reading and publishing without robot control
"""

import rclpy
from piper_teleop.xr_interface import XRInterface


def main(args=None):
    rclpy.init(args=args)

    # Create XR interface node (will auto-detect or use mock mode)
    xr_node = XRInterface(mode='auto')

    xr_node.get_logger().info('=== XR Interface Test Node ===')
    xr_node.get_logger().info('Publishing XR controller poses to:')
    xr_node.get_logger().info('  - /xr/right_controller/pose')
    xr_node.get_logger().info('  - /xr/left_controller/pose')
    xr_node.get_logger().info('  - /xr/headset/pose')
    xr_node.get_logger().info('')
    xr_node.get_logger().info('Monitor with: ros2 topic echo /xr/right_controller/pose')
    xr_node.get_logger().info('Press Ctrl+C to stop')

    try:
        rclpy.spin(xr_node)
    except KeyboardInterrupt:
        xr_node.get_logger().info('XR Interface test stopped')
    finally:
        xr_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
