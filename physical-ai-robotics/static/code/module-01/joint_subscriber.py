#!/usr/bin/env python3
"""
Joint Command Subscriber for Humanoid Robot

This node subscribes to joint commands and simulates actuator control
for a humanoid robot's motor controllers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSubscriber(Node):
    """Subscribes to joint commands and logs actuation."""
    
    def __init__(self):
        super().__init__('joint_subscriber')
        
        # Subscribe to /joint_commands topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_commands',
            self.joint_command_callback,
            10  # QoS depth
        )
        
        self.get_logger().info('Joint Subscriber started - Listening for commands')
    
    def joint_command_callback(self, msg):
        """
        Callback function executed when a JointState message is received.
        
        In a real robot, this would:
        1. Validate commands (safety limits)
        2. Send PWM signals to motor drivers
        3. Update internal state tracking
        """
        # Log received command
        self.get_logger().info('Received joint command:')
        
        # Iterate through all joints in the message
        for i, joint_name in enumerate(msg.name):
            # Extract position command (if present)
            position = msg.position[i] if i < len(msg.position) else None
            velocity = msg.velocity[i] if i < len(msg.velocity) else None
            effort = msg.effort[i] if i < len(msg.effort) else None
            
            # Log command for this joint
            cmd_str = f'  Joint: {joint_name}'
            if position is not None:
                cmd_str += f', Position: {position:.3f} rad'
            if velocity is not None:
                cmd_str += f', Velocity: {velocity:.3f} rad/s'
            if effort is not None:
                cmd_str += f', Effort: {effort:.3f} Nm'
            
            self.get_logger().info(cmd_str)
        
        # Simulate actuator response
        self.actuate_joints(msg)
    
    def actuate_joints(self, msg):
        """
        Simulate sending commands to hardware motor controllers.
        
        In a real system, this would interface with:
        - Dynamixel servos (via USB/RS485)
        - BLDC motor drivers (via CAN bus)
        - Pneumatic actuators (via control valves)
        """
        # Placeholder for hardware interface
        # In real code: send PWM, CAN messages, or servo protocol commands
        self.get_logger().debug('Actuating motors (simulated)')


def main(args=None):
    rclpy.init(args=args)
    joint_subscriber = JointSubscriber()
    
    try:
        rclpy.spin(joint_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        joint_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
