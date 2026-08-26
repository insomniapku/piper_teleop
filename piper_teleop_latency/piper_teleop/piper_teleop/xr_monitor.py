#!/usr/bin/env python3
"""
XR Data Monitor - Display all XR controller data fields
"""

import rclpy
from piper_teleop.xr_interface import XRInterface
import time


def main():
    rclpy.init()

    xr_node = XRInterface(mode='auto')

    print("=" * 70)
    print("XR DATA MONITOR")
    print("=" * 70)
    print(f"Mode: {xr_node.mode.upper()}")
    print("Press Ctrl+C to stop")
    print("=" * 70)
    print()

    try:
        rate = xr_node.create_rate(10)  # 10 Hz display
        while rclpy.ok():
            rclpy.spin_once(xr_node, timeout_sec=0)

            # Clear screen
            print("\033[2J\033[H", end="")

            # Header
            print("=" * 70)
            print(f"XR DATA MONITOR - {time.strftime('%H:%M:%S')}")
            print("=" * 70)

            # Right controller
            right = xr_node.get_right_controller()
            if right:
                print("\n[RIGHT CONTROLLER]")
                print(f"  Position:    [{right.position[0]:7.3f}, {right.position[1]:7.3f}, {right.position[2]:7.3f}]")
                print(f"  Quaternion:  [{right.quaternion[0]:6.3f}, {right.quaternion[1]:6.3f}, {right.quaternion[2]:6.3f}, {right.quaternion[3]:6.3f}]")
                print(f"  Grip:        {right.grip_value:.3f}  {'[ACTIVE]' if right.grip_value > 0.9 else ''}")
                print(f"  Trigger:     {right.trigger_value:.3f}")
                print(f"  Tracking:    {'✓ VALID' if right.tracking_valid else '✗ LOST'}")
                print(f"  Timestamp:   {right.timestamp_ns}")
            else:
                print("\n[RIGHT CONTROLLER] - No data")

            # Left controller
            left = xr_node.get_left_controller()
            if left:
                print("\n[LEFT CONTROLLER]")
                print(f"  Position:    [{left.position[0]:7.3f}, {left.position[1]:7.3f}, {left.position[2]:7.3f}]")
                print(f"  Quaternion:  [{left.quaternion[0]:6.3f}, {left.quaternion[1]:6.3f}, {left.quaternion[2]:6.3f}, {left.quaternion[3]:6.3f}]")
                print(f"  Grip:        {left.grip_value:.3f}")
                print(f"  Trigger:     {left.trigger_value:.3f}")
                print(f"  Tracking:    {'✓ VALID' if left.tracking_valid else '✗ LOST'}")
            else:
                print("\n[LEFT CONTROLLER] - No data")

            # Headset
            headset = xr_node.get_headset()
            if headset:
                print("\n[HEADSET]")
                print(f"  Position:    [{headset.position[0]:7.3f}, {headset.position[1]:7.3f}, {headset.position[2]:7.3f}]")
                print(f"  Quaternion:  [{headset.quaternion[0]:6.3f}, {headset.quaternion[1]:6.3f}, {headset.quaternion[2]:6.3f}, {headset.quaternion[3]:6.3f}]")
                print(f"  Tracking:    {'✓ VALID' if headset.tracking_valid else '✗ LOST'}")

            print("\n" + "=" * 70)
            print("Topics: /xr/right_controller/pose, /xr/left_controller/pose, /xr/headset/pose")
            print("=" * 70)

            rate.sleep()

    except KeyboardInterrupt:
        print("\n\nMonitor stopped")
    finally:
        xr_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
