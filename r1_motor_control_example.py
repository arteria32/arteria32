#!/usr/bin/env python3
"""
Galaxea R1 Pro Motor Control Example

Before running this script, source the R1 Pro workspace:
    source ~/galaxea_r1_sdk/install/setup.bash
    python3 r1_motor_control_example.py
"""

import rclpy
from rclpy.node import Node

# Import the custom message type from hdas_msg package
from hdas_msg.msg import MotorControl


class R1MotorController(Node):
    """Node to control R1 Pro arms and grippers."""

    def __init__(self):
        super().__init__('r1_motor_controller')

        # Create publishers for each motor control topic
        self.pub_arm_left = self.create_publisher(
            MotorControl,
            '/motion_control/control_arm_left',
            10
        )
        self.pub_arm_right = self.create_publisher(
            MotorControl,
            '/motion_control/control_arm_right',
            10
        )
        self.pub_gripper_left = self.create_publisher(
            MotorControl,
            '/motion_control/control_gripper_left',
            10
        )
        self.pub_gripper_right = self.create_publisher(
            MotorControl,
            '/motion_control/control_gripper_right',
            10
        )

        self.get_logger().info('R1 Motor Controller initialized')

    def send_arm_command(self, side: str, positions: list, velocities: list = None):
        """
        Send position command to arm.
        
        Args:
            side: 'left' or 'right'
            positions: List of joint positions (radians)
            velocities: Optional list of joint velocities
        """
        msg = MotorControl()
        
        # Set positions (adjust field names based on actual message definition)
        msg.position = positions
        
        if velocities:
            msg.velocity = velocities
        
        # Publish to appropriate topic
        if side == 'left':
            self.pub_arm_left.publish(msg)
            self.get_logger().info(f'Sent left arm command: {positions}')
        elif side == 'right':
            self.pub_arm_right.publish(msg)
            self.get_logger().info(f'Sent right arm command: {positions}')

    def send_gripper_command(self, side: str, position: float):
        """
        Send command to gripper.
        
        Args:
            side: 'left' or 'right'
            position: Gripper position (0.0 = closed, 1.0 = open, typically)
        """
        msg = MotorControl()
        msg.position = [position]
        
        if side == 'left':
            self.pub_gripper_left.publish(msg)
            self.get_logger().info(f'Sent left gripper command: {position}')
        elif side == 'right':
            self.pub_gripper_right.publish(msg)
            self.get_logger().info(f'Sent right gripper command: {position}')


def main():
    rclpy.init()
    
    controller = R1MotorController()
    
    try:
        # Example: Send commands
        # Adjust joint values based on R1 Pro specifications
        
        # Move left arm (example: 7 DOF arm)
        left_arm_positions = [0.0, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0]
        controller.send_arm_command('left', left_arm_positions)
        
        # Open left gripper
        controller.send_gripper_command('left', 1.0)
        
        # Keep node alive to process callbacks
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
