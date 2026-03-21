#!/usr/bin/env python3
"""
Standalone Trajectory Plotter (ROS 2)

Run alongside the controller and trajectory publisher to independently
log and plot the drone's tracking performance.

Subscribes: /goal_pose (PoseStamped)
Uses TF:    map -> crazyflie_real (configurable)

On Ctrl+C saves:
  - CSV:  trajectory_log.csv
  - Plot: traj_xyz_timeplots.png
  - Plot: traj_xy_traj.png
"""

import atexit
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class SimplePlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        # --- Parameters ---
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'crazyflie_real')
        self.declare_parameter('sample_hz', 20.0)
        self.declare_parameter('tf_timeout_sec', 0.10)
        self.declare_parameter('csv_path', 'trajectory_log.csv')
        self.declare_parameter('save_figs', True)
        self.declare_parameter('fig_prefix', 'traj')

        self.goal_topic   = self.get_parameter('goal_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.sample_hz    = float(self.get_parameter('sample_hz').value)
        self.tf_timeout   = float(self.get_parameter('tf_timeout_sec').value)
        self.csv_path     = self.get_parameter('csv_path').value
        self.save_figs    = bool(self.get_parameter('save_figs').value)
        self.fig_prefix   = self.get_parameter('fig_prefix').value

        # --- ROS interfaces ---
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)

        # --- State ---
        self.latest_goal = None
        self.t0 = self.get_clock().now()
        self.t, self.x, self.y, self.z = [], [], [], []
        self.gx, self.gy, self.gz = [], [], []

        # --- Timer ---
        self.create_timer(1.0 / max(self.sample_hz, 1.0), self._sample_once)
        atexit.register(self._finalize)

        self.get_logger().info(
            f"Plotter: {self.goal_topic} | TF: {self.target_frame} <- {self.source_frame} | "
            f"{self.sample_hz:.1f} Hz"
        )

    def _on_goal(self, msg: PoseStamped):
        self.latest_goal = msg

    def _sample_once(self):
        try:
            tf = self.tf_buf.lookup_transform(
                self.target_frame, self.source_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException:
            return

        now = self.get_clock().now()
        t_s = (now - self.t0).nanoseconds * 1e-9

        px = tf.transform.translation.x
        py = tf.transform.translation.y
        pz = tf.transform.translation.z

        if self.latest_goal is not None:
            gx = self.latest_goal.pose.position.x
            gy = self.latest_goal.pose.position.y
            gz = self.latest_goal.pose.position.z
        else:
            gx = gy = gz = 0.0

        self.t.append(t_s)
        self.x.append(px); self.y.append(py); self.z.append(pz)
        self.gx.append(gx); self.gy.append(gy); self.gz.append(gz)

    def _finalize(self):
        if not self.t:
            return

        # --- CSV ---
        arr = np.column_stack([self.t, self.x, self.y, self.z, self.gx, self.gy, self.gz])
        np.savetxt(self.csv_path, arr, delimiter=",",
                   header="time_s,x,y,z,gx,gy,gz", comments="")
        self.get_logger().info(f"Saved CSV: {self.csv_path} ({arr.shape[0]} samples)")

        if not self.save_figs:
            return

        # --- XYZ time series ---
        fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        ax1.plot(self.t, self.x, label="x (actual)")
        ax1.plot(self.t, self.gx, '--', label="x (goal)")
        ax2.plot(self.t, self.y, label="y (actual)")
        ax2.plot(self.t, self.gy, '--', label="y (goal)")
        ax3.plot(self.t, self.z, label="z (actual)")
        ax3.plot(self.t, self.gz, '--', label="z (goal)")
        ax1.set_ylabel("x [m]"); ax2.set_ylabel("y [m]"); ax3.set_ylabel("z [m]")
        ax3.set_xlabel("time [s]")
        for ax in (ax1, ax2, ax3):
            ax.legend(); ax.grid(True)
        fig1.tight_layout()
        f1 = f"{self.fig_prefix}_xyz_timeplots.png"
        fig1.savefig(f1, dpi=150); plt.close(fig1)
        self.get_logger().info(f"Saved plot: {f1}")

        # --- XY path ---
        xy = np.column_stack([self.x, self.y])
        gxy = np.column_stack([self.gx, self.gy])
        if len(gxy) > 1:
            changes = np.any(gxy[1:] != gxy[:-1], axis=1)
            gxy_show = gxy[np.concatenate([[True], changes])]
        else:
            gxy_show = gxy

        fig2, ax = plt.subplots(figsize=(7, 7))
        ax.plot(xy[:, 0], xy[:, 1], label="path (actual)")
        if len(gxy_show) > 0:
            ax.scatter(gxy_show[:, 0], gxy_show[:, 1], marker="x", s=100,
                      label="goals", zorder=5)
        ax.set_aspect('equal'); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.3); ax.legend()
        fig2.tight_layout()
        f2 = f"{self.fig_prefix}_xy_traj.png"
        fig2.savefig(f2, dpi=150); plt.close(fig2)
        self.get_logger().info(f"Saved plot: {f2}")


def main(args=None):
    rclpy.init(args=args)
    node = SimplePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
