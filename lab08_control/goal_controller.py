#!/usr/bin/env python3
"""
PID Goal Controller for Crazyflie (Real Hardware)

Based on the flight-proven 3d_goal_control.py, upgraded with YAML parameter
loading so students can set PID gains in config/pid_gains.yaml instead of
editing source code.

Subscribes: /goal_pose (PoseStamped)
Publishes:  /cmd_vel   (Twist)
Uses TF:    map -> crazyflie/base_footprint (configurable)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf_transformations
from rcl_interfaces.msg import SetParametersResult


class PIDController:
    """Simple PID with output clamping and integral windup protection."""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, min_output=None, max_output=None,
                 max_integral=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        self.max_integral = max_integral
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt

        # Anti-windup: clamp integral term
        if self.max_integral is not None:
            self.integral = max(-self.max_integral, min(self.max_integral, self.integral))

        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        return output


class GoalController(Node):
    def __init__(self):
        super().__init__('goal_controller')

        # --- Declare all parameters with defaults ---
        # TF frames
        self.declare_parameter('tf_world_frame', 'map')
        self.declare_parameter('tf_robot_frame', 'crazyflie/base_footprint')

        # Control
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('position_tolerance', 0.01)
        self.declare_parameter('yaw_tolerance', 0.01)

        # PID gains - X axis
        self.declare_parameter('pid_x.kp', 2.5)
        self.declare_parameter('pid_x.ki', 0.0)
        self.declare_parameter('pid_x.kd', 0.05)
        self.declare_parameter('pid_x.min_output', -0.707)
        self.declare_parameter('pid_x.max_output', 0.707)

        # PID gains - Y axis
        self.declare_parameter('pid_y.kp', 2.5)
        self.declare_parameter('pid_y.ki', 0.0)
        self.declare_parameter('pid_y.kd', 0.05)
        self.declare_parameter('pid_y.min_output', -0.707)
        self.declare_parameter('pid_y.max_output', 0.707)

        # PID gains - Z axis
        self.declare_parameter('pid_z.kp', 2.5)
        self.declare_parameter('pid_z.ki', 0.01)
        self.declare_parameter('pid_z.kd', 0.05)
        self.declare_parameter('pid_z.min_output', -0.4)
        self.declare_parameter('pid_z.max_output', 0.4)

        # PID gains - Yaw
        self.declare_parameter('pid_yaw.kp', 1.0)
        self.declare_parameter('pid_yaw.ki', 0.0)
        self.declare_parameter('pid_yaw.kd', 0.05)
        self.declare_parameter('pid_yaw.min_output', -1.0)
        self.declare_parameter('pid_yaw.max_output', 1.0)

        # --- Read parameters ---
        self.tf_world_frame = self.get_parameter('tf_world_frame').value
        self.tf_robot_frame = self.get_parameter('tf_robot_frame').value
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)

        # --- Build PID controllers from parameters ---
        self.pid_x = self._make_pid('pid_x')
        self.pid_y = self._make_pid('pid_y')
        self.pid_z = self._make_pid('pid_z')
        self.pid_yaw = self._make_pid('pid_yaw')

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # --- TF2 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ---
        self.goal_pose = None
        self.last_time = self.get_clock().now()
        self._last_tf_warning_time = None

        # --- Runtime parameter updates ---
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # --- Control loop timer ---
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            f"Goal controller initialized. "
            f"TF: {self.tf_world_frame} -> {self.tf_robot_frame}, "
            f"rate: {self.control_rate} Hz"
        )
        self._log_gains()

    def _make_pid(self, prefix):
        """Build a PIDController from ROS parameters with the given prefix."""
        return PIDController(
            kp=float(self.get_parameter(f'{prefix}.kp').value),
            ki=float(self.get_parameter(f'{prefix}.ki').value),
            kd=float(self.get_parameter(f'{prefix}.kd').value),
            min_output=float(self.get_parameter(f'{prefix}.min_output').value),
            max_output=float(self.get_parameter(f'{prefix}.max_output').value),
        )

    def _log_gains(self):
        """Log current PID gains for visibility."""
        for name, pid in [('X', self.pid_x), ('Y', self.pid_y),
                          ('Z', self.pid_z), ('Yaw', self.pid_yaw)]:
            self.get_logger().info(
                f"  {name}: kp={pid.kp:.4f}  ki={pid.ki:.4f}  kd={pid.kd:.4f}  "
                f"out=[{pid.min_output}, {pid.max_output}]"
            )

    @staticmethod
    def _poses_match(lhs, rhs, tol=1e-6):
        """Treat repeated publications of the same goal as one target."""
        return (
            math.isclose(lhs.position.x, rhs.position.x, abs_tol=tol) and
            math.isclose(lhs.position.y, rhs.position.y, abs_tol=tol) and
            math.isclose(lhs.position.z, rhs.position.z, abs_tol=tol) and
            math.isclose(lhs.orientation.x, rhs.orientation.x, abs_tol=tol) and
            math.isclose(lhs.orientation.y, rhs.orientation.y, abs_tol=tol) and
            math.isclose(lhs.orientation.z, rhs.orientation.z, abs_tol=tol) and
            math.isclose(lhs.orientation.w, rhs.orientation.w, abs_tol=tol)
        )

    def _on_parameter_change(self, params):
        """Handle runtime parameter updates (e.g., from ros2 param set)."""
        changed = []
        for p in params:
            name = p.name
            val = p.value

            if name.startswith('pid_') and '.' in name:
                axis, field = name.split('.', 1)
                pid_map = {
                    'pid_x': self.pid_x, 'pid_y': self.pid_y,
                    'pid_z': self.pid_z, 'pid_yaw': self.pid_yaw
                }
                target_pid = pid_map.get(axis)
                if target_pid and val is not None:
                    if field in ('kp', 'ki', 'kd', 'min_output', 'max_output'):
                        setattr(target_pid, field, float(val))
                        changed.append(f"{name}={float(val):.4f}")

        if changed:
            self.get_logger().info(f"Parameters updated: {', '.join(changed)}")

        return SetParametersResult(successful=True)

    # --- Callbacks ---

    def goal_callback(self, msg: PoseStamped):
        is_new_goal = self.goal_pose is None or not self._poses_match(self.goal_pose, msg.pose)
        self.goal_pose = msg.pose
        if not is_new_goal:
            return

        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self.last_time = self.get_clock().now()
        self.get_logger().info(
            f"New goal: x={msg.pose.position.x:.2f}, "
            f"y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}"
        )

    def control_loop(self):
        if self.goal_pose is None:
            return

        # Look up current robot pose via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                self.tf_world_frame, self.tf_robot_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            now = self.get_clock().now()
            if (self._last_tf_warning_time is None or
                    (now - self._last_tf_warning_time).nanoseconds >= 2_000_000_000):
                self.get_logger().warning(f"TF lookup failed: {e}")
                self._last_tf_warning_time = now
            return

        # Compute dt
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        # Extract current pose
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z
        q = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Extract goal pose
        gx = self.goal_pose.position.x
        gy = self.goal_pose.position.y
        gz = self.goal_pose.position.z
        gq = self.goal_pose.orientation
        _, _, goal_yaw = tf_transformations.euler_from_quaternion([gq.x, gq.y, gq.z, gq.w])

        # Compute errors
        dx = gx - x
        dy = gy - y
        dz = gz - z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        yaw_error = math.atan2(math.sin(goal_yaw - yaw), math.cos(goal_yaw - yaw))

        # Goal reached check
        if distance < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.get_logger().info("Goal reached.")
            self.goal_pose = None
            self.cmd_pub.publish(Twist())
            return

        # PID in world frame
        vx_world = self.pid_x.compute(dx, dt)
        vy_world = self.pid_y.compute(dy, dt)
        vz_world = self.pid_z.compute(dz, dt)
        wz = self.pid_yaw.compute(yaw_error, dt)

        # Rotate XY velocity into robot body frame
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_robot = cos_yaw * vx_world + sin_yaw * vy_world
        vy_robot = -sin_yaw * vx_world + cos_yaw * vy_world

        # Publish
        cmd = Twist()
        cmd.linear.x = vx_robot
        cmd.linear.y = vy_robot
        cmd.linear.z = vz_world
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
