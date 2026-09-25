#!/usr/bin/env python3
"""
Main Teleoperation Node (M1 - Basic Structure)
Coordinates XR input, mapping, IK, and robot control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PiperTeleopNode(Node):
    """
    Main teleoperation node
    """

    def __init__(self):
        super().__init__('piper_teleop_node')

        # Load configuration
        self.config = self.load_config()

        # Log startup
        self.get_logger().info('=== Piper Teleoperation Node Started (M1) ===')
        self.get_logger().info('Package structure created successfully')
        self.get_logger().info('Configuration loaded')

        # Placeholders for future modules (M2-M8)
        self.xr_interface = None  # M2: XR input
        self.teleop_mapper = None  # M3: Delta mapping
        self.ik_solver = None  # M4: IK solver
        self.piper_interface = None  # M6: Piper hardware interface

        # Publishers (placeholder)
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/piper_teleop/target_pose',
            10
        )

        self.joint_target_pub = self.create_publisher(
            JointState,
            '/piper_teleop/joint_target',
            10
        )

        # Timer for main control loop (placeholder - will be activated in later stages)
        self.control_timer = self.create_timer(
            1.0 / self.config['debug']['publish_rate'],
            self.control_loop_callback
        )

        self.get_logger().info('M1 Complete: Package structure ready')
        self.get_logger().info('Next: M2 - Implement XR data interface')

    def load_config(self) -> dict:
        """Load configuration from YAML file"""
        try:
            # Try to load from package share directory
            pkg_dir = get_package_share_directory('piper_teleop')
            config_path = os.path.join(pkg_dir, 'config', 'teleop_config.yaml')
        except:
            # Fallback to local path during development
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'config',
                'teleop_config.yaml'
            )

        self.get_logger().info(f'Loading config from: {config_path}')

        if not os.path.exists(config_path):
            self.get_logger().error(f'Config file not found: {config_path}')
            # Return default config
            return self.get_default_config()

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.get_logger().info('Configuration loaded successfully')

        # Extract ros__parameters if present (ROS 2 YAML format)
        if 'piper_teleop_node' in config and 'ros__parameters' in config['piper_teleop_node']:
            config = config['piper_teleop_node']['ros__parameters']

        return config

    def get_default_config(self) -> dict:
        """Return default configuration"""
        return {
            'position_mapping': {'scale': 1.0, 'max_delta': 0.1},
            'rotation_mapping': {'scale': 1.0, 'max_delta': 0.2},
            'xr_to_piper_transform': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0]
            },
            'debug': {'publish_rate': 50}
        }

    def control_loop_callback(self):
        """Main control loop (placeholder for M1)"""
        # This will be implemented in stages M2-M8
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PiperTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down teleoperation node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
