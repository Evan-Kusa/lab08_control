#!/usr/bin/env python3
"""
Trajectory Publisher and Mission Logger

Publishes a sequence of waypoints to /goal_pose, monitors progress via TF,
and logs actual vs goal poses. On completion (or Ctrl+C), saves:
  - CSV with timestamped pose data
  - XYZ time-series plot
  - XY path plot

Waypoints are loaded from a YAML file (config/waypoints.yaml).
"""

import atexit
import math
import time
import os
import yaml
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import matplotlib.ticker as mticker

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_waypoints_from_yaml(path):
    """
    Load waypoints from a YAML file. Expected format:

    waypoints:
      - [x, y, z, yaw]
      - [x, y, z, yaw]
      ...
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or 'waypoints' not in data:
        raise ValueError(f"YAML file must contain a 'waypoints' key: {path}")

    waypoints = []
    for i, wp in enumerate(data['waypoints']):
        if not isinstance(wp, (list, tuple)) or len(wp) != 4:
            raise ValueError(f"Waypoint {i} must be [x, y, z, yaw], got: {wp}")
        waypoints.append(tuple(float(v) for v in wp))

    return waypoints


class MissionControllerAndLogger(Node):
    """Publishes waypoint goals and logs robot pose vs. goal; saves CSV + plots on completion."""

    def __init__(self):
        super().__init__('mission_controller')

        # --- Parameters ---
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('goal_timeout', 15.0)
        self.declare_parameter('goal_tolerance_xy', 0.05)
        self.declare_parameter('goal_tolerance_z', 0.05)
        self.declare_parameter('goal_tolerance_yaw', 0.1)
        self.declare_parameter('robot_world_frame', 'map')
        self.declare_parameter('robot_base_frame', 'crazyflie_real')
        self.declare_parameter('sample_hz', 20.0)
        self.declare_parameter('csv_filename', 'trajectory_log.csv')
        self.declare_parameter('plot_filename_prefix', 'traj')
        self.declare_parameter('waypoints_file', '')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.goal_tolerance_xy = float(self.get_parameter('goal_tolerance_xy').value)
        self.goal_tolerance_z = float(self.get_parameter('goal_tolerance_z').value)
        self.goal_tolerance_yaw = float(self.get_parameter('goal_tolerance_yaw').value)
        self.robot_world_frame = self.get_parameter('robot_world_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.sample_hz = float(self.get_parameter('sample_hz').value)
        self.csv_filename = self.get_parameter('csv_filename').value
        self.plot_filename_prefix = self.get_parameter('plot_filename_prefix').value
        waypoints_file = self.get_parameter('waypoints_file').value

        # --- Load waypoints ---
        if waypoints_file:
            try:
                self.pose_sequence = load_waypoints_from_yaml(waypoints_file)
                self.get_logger().info(f"Loaded {len(self.pose_sequence)} waypoints from {waypoints_file}")
            except Exception as e:
                self.get_logger().error(f"Failed to load waypoints from '{waypoints_file}': {e}")
                self.get_logger().warn("Using default waypoints.")
                self.pose_sequence = self._default_waypoints()
        else:
            self.get_logger().info("No waypoints_file specified, using defaults.")
            self.pose_sequence = self._default_waypoints()

        # --- Publishers ---
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.hover_publisher = self.create_publisher(Float32, '/hover_height', 10)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- State ---
        self.current_goal_index = 0
        self.active_goal = None
        self.goal_start_time = 0.0

        # --- Logging buffers ---
        self.log_t0 = self.get_clock().now()
        self.log_time, self.log_x, self.log_y, self.log_z = [], [], [], []
        self.log_gx, self.log_gy, self.log_gz = [], [], []
        self._is_finalized = False

        # --- Timers ---
        self.goal_timer = self.create_timer(1.0 / max(1e-6, self.publish_rate), self._goal_timer_callback)
        self.logging_timer = self.create_timer(1.0 / max(1e-6, self.sample_hz), self._logging_timer_callback)

        # --- Safe shutdown ---
        atexit.register(self._finalize_safe)

        self._safe_log(
            f"Mission controller initialized. "
            f"frame='{self.robot_world_frame}', {len(self.pose_sequence)} waypoints, "
            f"publish {self.publish_rate} Hz, log {self.sample_hz} Hz"
        )
        self.publish_next_goal()

    @staticmethod
    def _default_waypoints():
        """Fallback waypoints matching the lab 8 physical course."""
        return [
            (0.0, 0.0, 0.3, 0.0),
            (0.5588, 0.0, 0.3, 0.0),
            (0.5588, 0.8128, 0.33, 0.0),
            (0.0, 0.8128, 0.33, 0.0),
            (0.0, -0.71, 0.33, 0.0),
            (-1.40, -0.71, 0.33, 0.0),
            (-1.40, -0.25, 0.33, 0.0),
            (-0.81, -0.25, 0.33, 0.0),
            (-0.81, -0.25, 1.0, 0.0),
            (-0.81, -0.95, 1.0, 0.0),
            (0.0, -0.95, 1.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.3, 0.0),
            (0.0, 0.0, -0.05, 0.0),
        ]

    # --- Goal publishing ---

    def publish_next_goal(self):
        if self.current_goal_index >= len(self.pose_sequence):
            self._safe_log("All waypoints completed.")
            self.active_goal = None
            self._start_shutdown()
            return

        x, y, z, yaw = self.pose_sequence[self.current_goal_index]
        self.active_goal = (x, y, z, yaw)
        self.goal_start_time = time.time()

        goal_msg = self._create_goal_message(x, y, z, yaw)
        self.goal_publisher.publish(goal_msg)

        z_display = 'nan' if math.isnan(z) else f'{z:.2f}'
        self._safe_log(
            f"Goal #{self.current_goal_index + 1}/{len(self.pose_sequence)}: "
            f"x={x:.2f}, y={y:.2f}, z={z_display}, yaw={yaw:.2f}"
        )
        self.current_goal_index += 1

    def _goal_timer_callback(self):
        if self.active_goal is None:
            return
        x, y, z, yaw = self.active_goal
        self.goal_publisher.publish(self._create_goal_message(x, y, z, yaw))
        if not math.isnan(z):
            self.hover_publisher.publish(Float32(data=float(z)))
        self._check_goal_status()

    # --- Logging ---

    def _logging_timer_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_world_frame, self.robot_base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException:
            return

        now = self.get_clock().now()
        t_s = (now - self.log_t0).nanoseconds * 1e-9
        self.log_time.append(t_s)
        self.log_x.append(tf.transform.translation.x)
        self.log_y.append(tf.transform.translation.y)
        self.log_z.append(tf.transform.translation.z)

        if self.active_goal is not None:
            gx, gy, gz, _ = self.active_goal
            self.log_gx.append(gx)
            self.log_gy.append(gy)
            self.log_gz.append(gz)
        else:
            self.log_gx.append(float('nan'))
            self.log_gy.append(float('nan'))
            self.log_gz.append(float('nan'))

    # --- Goal checking ---

    def _check_goal_status(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_world_frame, self.robot_base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            pos = transform.transform.translation
            rot = transform.transform.rotation
            _, _, current_yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            if self._is_goal_reached(pos, current_yaw):
                self._safe_log("Goal reached. Advancing.")
                self.publish_next_goal()
            elif time.time() - self.goal_start_time > self.goal_timeout:
                self.get_logger().warning("Goal timeout. Advancing.")
                self.publish_next_goal()

        except TransformException:
            pass

    def _is_goal_reached(self, current_pos, current_yaw):
        goal_x, goal_y, goal_z, goal_yaw = self.active_goal
        dx = goal_x - current_pos.x
        dy = goal_y - current_pos.y
        dz = 0.0 if math.isnan(goal_z) else goal_z - current_pos.z
        dyaw = math.atan2(math.sin(goal_yaw - current_yaw), math.cos(goal_yaw - current_yaw))
        is_z_ok = math.isnan(goal_z) or (abs(dz) < self.goal_tolerance_z)

        return (abs(dx) < self.goal_tolerance_xy and
                abs(dy) < self.goal_tolerance_xy and
                is_z_ok and
                abs(dyaw) < self.goal_tolerance_yaw)

    def _create_goal_message(self, x, y, z, yaw):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.robot_world_frame
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.position.z = float(z)
        q = quaternion_from_euler(0, 0, float(yaw))
        goal_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return goal_msg

    # --- Finalization ---

    def _safe_log(self, msg, level="info"):
        try:
            if rclpy.ok():
                getattr(self.get_logger(), level)(msg)
            else:
                print(msg)
        except Exception:
            print(msg)

    def _finalize(self):
        if self._is_finalized or not self.log_time:
            return
        self._is_finalized = True
        self._safe_log("Saving logs and generating plots...")

        data = np.column_stack([
            self.log_time, self.log_x, self.log_y, self.log_z,
            self.log_gx, self.log_gy, self.log_gz
        ])

        # --- CSV ---
        try:
            csv_path = os.path.abspath(self.csv_filename)
            with open(self.csv_filename, 'w') as f:
                f.write("time_s,x,y,z,gx,gy,gz\n")
                for row in data:
                    f.write(",".join(map(str, row)) + "\n")
            self._safe_log(f"Saved CSV: {csv_path} ({data.shape[0]} samples)")
        except Exception as e:
            self._safe_log(f"Failed to save CSV: {e}", level="error")

        # --- Compute error metrics ---
        try:
            t = np.array(self.log_time)
            actual = np.column_stack([self.log_x, self.log_y, self.log_z])
            goal = np.column_stack([self.log_gx, self.log_gy, self.log_gz])
            valid = ~np.isnan(goal).any(axis=1)

            if valid.any():
                errors = actual[valid] - goal[valid]
                pos_errors = np.linalg.norm(errors, axis=1)
                rms_error = np.sqrt(np.mean(pos_errors**2))
                max_error = np.max(pos_errors)
                mean_error = np.mean(pos_errors)

                # Per-axis RMS
                rms_x = np.sqrt(np.mean(errors[:, 0]**2))
                rms_y = np.sqrt(np.mean(errors[:, 1]**2))
                rms_z = np.sqrt(np.mean(errors[:, 2]**2))

                total_time = t[-1] - t[0] if len(t) > 1 else 0.0

                self._safe_log(
                    f"=== Performance Metrics ===\n"
                    f"  Total time:     {total_time:.1f} s\n"
                    f"  RMS error:      {rms_error:.4f} m\n"
                    f"  Max error:      {max_error:.4f} m\n"
                    f"  Mean error:     {mean_error:.4f} m\n"
                    f"  RMS error (X):  {rms_x:.4f} m\n"
                    f"  RMS error (Y):  {rms_y:.4f} m\n"
                    f"  RMS error (Z):  {rms_z:.4f} m"
                )
        except Exception as e:
            self._safe_log(f"Error computing metrics: {e}", level="error")

        # --- Time-series plot ---
        try:
            fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            t = np.array(self.log_time)

            ax1.plot(t, self.log_x, label="Actual X")
            ax1.plot(t, self.log_gx, 'r--', label="Goal X")
            ax1.set_ylabel("X [m]"); ax1.legend(); ax1.grid(True)

            ax2.plot(t, self.log_y, label="Actual Y")
            ax2.plot(t, self.log_gy, 'r--', label="Goal Y")
            ax2.set_ylabel("Y [m]"); ax2.legend(); ax2.grid(True)

            ax3.plot(t, self.log_z, label="Actual Z")
            ax3.plot(t, self.log_gz, 'r--', label="Goal Z")
            ax3.set_ylabel("Z [m]"); ax3.set_xlabel("Time [s]")
            ax3.legend(); ax3.grid(True)

            if t.size > 1:
                ax3.xaxis.set_major_locator(mticker.MaxNLocator(nbins=10, prune='both'))
                major_ticks = ax3.xaxis.get_majorticklocs()
                if len(major_ticks) >= 2:
                    interval = major_ticks[1] - major_ticks[0]
                    ax3.xaxis.set_minor_locator(mticker.MultipleLocator(interval / 2.0))
                ax3.set_xlim(float(t.min()), float(t.max()))

            fig1.tight_layout()
            plot1 = f"{self.plot_filename_prefix}_xyz_time.png"
            fig1.savefig(plot1, dpi=300); plt.close(fig1)
            self._safe_log(f"Saved time-series plot: {os.path.abspath(plot1)}")
        except Exception as e:
            self._safe_log(f"Failed to generate time plot: {e}", level="error")

        # --- XY path plot ---
        try:
            fig2, ax = plt.subplots(figsize=(8, 8))
            ax.plot(self.log_x, self.log_y, label="Actual Path")
            unique_goals = np.unique(np.column_stack([self.log_gx, self.log_gy]), axis=0)
            unique_goals = unique_goals[~np.isnan(unique_goals).any(axis=1)]
            if unique_goals.size > 0:
                ax.scatter(unique_goals[:, 0], unique_goals[:, 1],
                          marker='x', s=100, label="Waypoints")
            ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
            ax.set_title("XY Trajectory"); ax.legend(); ax.grid(True)
            ax.set_aspect('equal', adjustable='box')
            fig2.tight_layout()
            plot2 = f"{self.plot_filename_prefix}_xy_path.png"
            fig2.savefig(plot2, dpi=300); plt.close(fig2)
            self._safe_log(f"Saved XY path plot: {os.path.abspath(plot2)}")
        except Exception as e:
            self._safe_log(f"Failed to generate XY plot: {e}", level="error")

    def _finalize_safe(self):
        try:
            self._finalize()
        except Exception as e:
            print(f"Exception during finalize: {e}")

    def _start_shutdown(self):
        try:
            self.goal_timer.cancel()
            self.logging_timer.cancel()
        except Exception:
            pass
        self._finalize()
        self._safe_log("Mission complete. Shutting down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MissionControllerAndLogger()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        if node:
            node._safe_log("Shutdown signal received.")
    finally:
        try:
            if node and not node._is_finalized:
                node._finalize()
        except Exception:
            pass
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
