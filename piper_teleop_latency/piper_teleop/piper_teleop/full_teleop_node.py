#!/usr/bin/env python3
"""
Complete Piper Teleoperation Node
Integrates XR input, mapping, IK, and robot control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from piper_teleop.teleop_mapping import TeleopMapper
from piper_teleop.ik_solver_placo import PiperIKSolverPlaco


class FullTeleopNode(Node):
    """Complete teleoperation node with all functionality"""

    def __init__(self):
        super().__init__('full_teleop_node')
        self.declare_parameter('urdf_path', os.environ.get('PIPER_URDF_PATH', ''))

        # Load configuration
        self.config = self.load_config()

        self.get_logger().info('='*60)
        self.get_logger().info('Piper Full Teleoperation Node Starting...')
        self.get_logger().info('='*60)

        # State variables for controller data
        self.right_controller_pose = None
        self.right_controller_position = None
        self.right_controller_quaternion = None

        # Grip and trigger values (will be updated from other topics if available)
        self.grip_value = 0.0
        self.trigger_value = 0.0

        # Subscribe to XR controller pose
        self.controller_sub = self.create_subscription(
            PoseStamped,
            '/xr/right_controller/pose',
            self.controller_callback,
            10
        )
        self.get_logger().info('✓ Subscribed to /xr/right_controller/pose')

        # Initialize teleoperation mapper
        self.mapper = TeleopMapper(self.config)
        self.get_logger().info('✓ Teleoperation Mapper initialized')

        urdf_path = self.get_parameter("urdf_path").value
        if not os.path.exists(urdf_path):
            message = (
                "URDF file not found. Set the urdf_path ROS parameter or "
                "PIPER_URDF_PATH to a Piper URDF file."
            )
            self.get_logger().error(f"{message} Received: {urdf_path or "<empty>"}")
            raise FileNotFoundError(message)

        self.ik_solver = PiperIKSolverPlaco(urdf_path)
        self.get_logger().info(f'✓ IK Solver initialized with URDF: {urdf_path}')

        # State variables
        self.grip_active = False
        self.last_grip_state = False
        self.reference_pose = None
        self.current_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.last_valid_joint_angles = self.current_joint_angles.copy()

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            self.config.get('piper', {}).get('joint_ctrl_topic', '/joint_ctrl_single'),
            10
        )

        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/piper_teleop/target_pose',
            10
        )

        self.gripper_pub = self.create_publisher(
            Float32,
            '/gripper_cmd',
            10
        )

        # Control loop timer
        control_rate = self.config.get('debug', {}).get('publish_rate', 50)
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )

        self.get_logger().info('='*60)
        self.get_logger().info('✓ Full Teleoperation Node Ready!')
        self.get_logger().info('  - Press GRIP to activate control')
        self.get_logger().info('  - Move controller to control robot')
        self.get_logger().info('  - Pull TRIGGER to control gripper')
        self.get_logger().info('='*60)

    def controller_callback(self, msg):
        """Callback for XR controller pose"""
        self.right_controller_pose = msg
        self.right_controller_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.right_controller_quaternion = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

    def load_config(self) -> dict:
        """Load configuration from YAML file"""
        try:
            pkg_dir = get_package_share_directory('piper_teleop')
            config_path = os.path.join(pkg_dir, 'config', 'teleop_config.yaml')
        except:
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'config',
                'teleop_config.yaml'
            )

        self.get_logger().info(f'Loading config from: {config_path}')

        if not os.path.exists(config_path):
            return self.get_default_config()

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Extract ros__parameters if present
        if 'piper_teleop_node' in config and 'ros__parameters' in config['piper_teleop_node']:
            config = config['piper_teleop_node']['ros__parameters']
        elif 'full_teleop_node' in config and 'ros__parameters' in config['full_teleop_node']:
            config = config['full_teleop_node']['ros__parameters']

        return config

    def get_default_config(self) -> dict:
        """Return default configuration"""
        return {
            'position_mapping': {'scale': 1.0, 'max_delta': 0.1},
            'rotation_mapping': {'scale': 1.0, 'max_delta': 0.2},
            'piper': {
                'joint_ctrl_topic': '/joint_ctrl_single',
                'joint_state_topic': '/joint_states_feedback',
            },
            'gripper': {
                'trigger_threshold': 0.5,
                'open_position': 0.0,
                'close_position': 0.08,
            },
            'debug': {'publish_rate': 50}
        }

    def control_loop(self):
        """Main control loop"""
        # Check if we have controller data
        if self.right_controller_position is None or self.right_controller_quaternion is None:
            return

        # Check grip button state (simulated with keyboard for now - need to add button topic subscription)
        # For now, auto-activate after first position received
        if not self.grip_active and self.reference_pose is None:
            # Auto-activate on first valid data
            ee_pos = self.get_current_ee_position()
            ee_quat = np.array([0.0, 0.0, 0.0, 1.0])

            # Set reference in mapper
            self.mapper.set_reference(
                self.right_controller_position,
                self.right_controller_quaternion,
                ee_pos,
                ee_quat
            )

            self.reference_pose = {
                'position': self.right_controller_position.copy(),
                'quaternion': self.right_controller_quaternion.copy(),
                'ee_position': ee_pos,
                'ee_quaternion': ee_quat,
            }
            self.grip_active = True
            self.get_logger().info('🟢 Control ACTIVATED - Move controller to control robot')

        # If grip is active, compute and send commands
        if self.grip_active and self.reference_pose is not None:
            # Compute target EE pose using mapper
            target_pose_msg = self.mapper.compute_target_pose(
                self.right_controller_position,
                self.right_controller_quaternion
            )

            if target_pose_msg is None:
                return

            # Convert Pose message to arrays
            target_position = np.array([
                target_pose_msg.position.x,
                target_pose_msg.position.y,
                target_pose_msg.position.z
            ])
            target_quaternion = np.array([
                target_pose_msg.orientation.x,
                target_pose_msg.orientation.y,
                target_pose_msg.orientation.z,
                target_pose_msg.orientation.w
            ])

            # Convert quaternion to rotation matrix for IK solver
            from scipy.spatial.transform import Rotation as R
            target_orientation_matrix = R.from_quat(target_quaternion).as_matrix()

            # Solve IK
            joint_solution = self.ik_solver.solve_ik(
                target_position,
                target_orientation_matrix,
                seed=self.last_valid_joint_angles
            )

            if joint_solution is not None:
                self.current_joint_angles = joint_solution
                self.last_valid_joint_angles = joint_solution.copy()

                # Publish joint command
                self.publish_joint_command(joint_solution)

                # Publish target pose for visualization
                target_pose_dict = {
                    'position': target_position,
                    'quaternion': target_quaternion
                }
                self.publish_target_pose(target_pose_dict)
            else:
                self.get_logger().warn('IK solution not found, holding last position', throttle_duration_sec=1.0)

        # Handle gripper control
        gripper_config = self.config.get('gripper', {})
        threshold = gripper_config.get('trigger_threshold', 0.5)

        if self.trigger_value > threshold:
            gripper_pos = gripper_config.get('close_position', 0.08)
        else:
            gripper_pos = gripper_config.get('open_position', 0.0)

        self.publish_gripper_command(gripper_pos)

        if self.trigger_value > threshold:
            gripper_pos = gripper_config.get('close_position', 0.08)
        else:
            gripper_pos = gripper_config.get('open_position', 0.0)

        self.publish_gripper_command(gripper_pos)

    def get_current_ee_position(self):
        """Get current end-effector position from FK"""
        # Use forward kinematics to get current EE position
        ee_position, ee_orientation = self.ik_solver.compute_fk(self.current_joint_angles)
        return ee_position

    def publish_joint_command(self, joint_angles):
        """Publish joint command"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = joint_angles.tolist()
        self.joint_cmd_pub.publish(msg)

    def publish_target_pose(self, target_pose):
        """Publish target pose for visualization"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = target_pose['position'][0]
        msg.pose.position.y = target_pose['position'][1]
        msg.pose.position.z = target_pose['position'][2]
        msg.pose.orientation.x = target_pose['quaternion'][0]
        msg.pose.orientation.y = target_pose['quaternion'][1]
        msg.pose.orientation.z = target_pose['quaternion'][2]
        msg.pose.orientation.w = target_pose['quaternion'][3]
        self.target_pose_pub.publish(msg)

    def publish_gripper_command(self, position):
        """Publish gripper command"""
        msg = Float32()
        msg.data = position
        self.gripper_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = FullTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
