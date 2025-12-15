#!/usr/bin/env python3
"""
IMU Publisher Node for Humanoid Robot Balance Control

This node simulates an IMU (Inertial Measurement Unit) sensor publishing
orientation and acceleration data for a humanoid robot's balance system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import time


class ImuPublisher(Node):
    """Publishes simulated IMU data for humanoid robot balance."""
    
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create publisher on /imu/data topic with QoS depth of 10
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Publish at 100 Hz (typical IMU rate for humanoid balance control)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # Simulation variables
        self.sequence = 0
        self.start_time = time.time()
        
        self.get_logger().info('IMU Publisher started - Publishing at 100 Hz')
    
    def timer_callback(self):
        """Publish simulated IMU data every 10ms."""
        msg = Imu()
        
        # Header with timestamp and frame ID
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Simulate slight tilt (robot leaning forward/backward)
        elapsed = time.time() - self.start_time
        tilt = 0.05 * math.sin(elapsed)  # +/- 0.05 rad oscillation
        
        # Orientation (quaternion) - simplified, represents small tilt
        # In real system, this comes from sensor fusion
        msg.orientation.x = tilt
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = math.sqrt(1.0 - tilt**2)
        
        # Angular velocity (rad/s) - robot is slowly tilting
        msg.angular_velocity.x = 0.05 * math.cos(elapsed)
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # Linear acceleration (m/s²) - includes gravity + motion
        msg.linear_acceleration.x = 0.1 * math.sin(elapsed)
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity
        
        # Publish the message
        self.publisher_.publish(msg)
        self.sequence += 1
        
        # Log every 100 messages (once per second at 100 Hz)
        if self.sequence % 100 == 0:
            self.get_logger().info(
                f'Published IMU data #{self.sequence} - '
                f'Tilt: {tilt:.3f} rad, '
                f'Accel Z: {msg.linear_acceleration.z:.2f} m/s²'
            )


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
