#!/usr/bin/env python3
"""
Velocity Mux with Takeoff/Land Logic

Sits between the goal controller (/cmd_vel) and the Crazyflie driver
({prefix}/cmd_vel). Handles:
  - Takeoff sequencing (ramp up to hover height)
  - Landing detection (low altitude -> stop)
  - Height hold when no Z command is given
  - Angular rate capping

This file is from the original crazyflie sim/real stack (Bitcraze/Kimberly McGuire).
"""

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class ControlServices(Node):
    def __init__(self):
        super().__init__('control_services')

        self.declare_parameter('hover_height', 0.5)
        self.declare_parameter('robot_prefix', '/crazyflie')
        self.declare_parameter('incoming_twist_topic', '/cmd_vel')
        self.declare_parameter('max_ang_z_rate', 5.0)

        hover_height = self.get_parameter('hover_height').value
        robot_prefix = self.get_parameter('robot_prefix').value
        incoming_twist_topic = self.get_parameter('incoming_twist_topic').value
        max_ang_z_rate = self.get_parameter('max_ang_z_rate').value

        self.publisher_ = self.create_publisher(Twist, robot_prefix + incoming_twist_topic, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odometry_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, incoming_twist_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.takeoff_command = False
        self.current_pose = Odometry().pose.pose
        self.takeoff_height = hover_height
        self.max_ang_z_rate = max_ang_z_rate
        self.is_flying = False
        self.keep_height = False
        self.teleop_cmd = Twist()

    def timer_callback(self):
        msg = self.teleop_cmd
        height_command = msg.linear.z
        new_cmd_msg = Twist()

        if self.is_flying:
            new_cmd_msg.linear.x = msg.linear.x
            new_cmd_msg.linear.y = msg.linear.y
            new_cmd_msg.linear.z = msg.linear.z
            new_cmd_msg.angular.x = msg.angular.x
            new_cmd_msg.angular.y = msg.angular.y
            new_cmd_msg.angular.z = msg.angular.z

        # Takeoff
        if height_command > 0 and not self.is_flying:
            new_cmd_msg.linear.z = 0.5
            if self.current_pose.position.z > self.takeoff_height:
                new_cmd_msg.linear.z = 0.0
                self.teleop_cmd.linear.z = 0.0
                self.is_flying = True
                self.get_logger().info('Takeoff completed')

        # Landing
        if height_command < 0 and self.is_flying:
            if self.current_pose.position.z < 0.1:
                new_cmd_msg.linear.z = 0.0
                self.is_flying = False
                self.keep_height = False
                self.get_logger().info('Landing completed')

        # Angular rate cap
        if abs(msg.angular.z) > self.max_ang_z_rate:
            new_cmd_msg.angular.z = self.max_ang_z_rate * abs(msg.angular.z) / msg.angular.z

        # Height hold
        tolerance = 1e-7
        if abs(height_command) < tolerance and self.is_flying:
            if not self.keep_height:
                self.desired_height = self.current_pose.position.z
                self.keep_height = True
            else:
                error = self.desired_height - self.current_pose.position.z
                new_cmd_msg.linear.z = error

        if abs(height_command) > tolerance and self.is_flying:
            if self.keep_height:
                self.keep_height = False

        self.publisher_.publish(new_cmd_msg)

    def odometry_callback(self, msg):
        self.current_pose = msg.pose.pose

    def cmd_vel_callback(self, msg):
        self.teleop_cmd = msg


def main(args=None):
    rclpy.init(args=args)
    control_services = ControlServices()
    rclpy.spin(control_services)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
