# Lab 08 Control — Crazyflie Real Drone

ROS 2 (Jazzy) package for Lab 08: flying a real Crazyflie drone through a waypoint course using student-tuned PID gains from Lab 07 (simulation).

## Prerequisites

- ROS 2 Jazzy
- [crazyswarm2](https://github.com/IMRCLab/crazyswarm2) installed and configured for your Crazyflie
- Python packages: `numpy`, `matplotlib`, `pyyaml`, `tf_transformations`

## Package Structure

```
lab08_control/
├── config/
│   ├── pid_defaults.yaml      # Known-good PID gains (instructor reference)
│   ├── pid_student.yaml       # Template for student gains (edit this)
│   └── waypoints.yaml         # Lab course waypoints
├── launch/
│   └── lab08.launch.py        # Launches everything with one command
├── lab08_control/
│   ├── goal_controller.py     # PID controller node
│   ├── trajectory_publisher.py # Waypoint sequencer + data logger
│   ├── plotter.py             # Standalone trajectory plotter
│   └── control_services.py    # Velocity mux (takeoff/land/height hold)
├── package.xml
├── setup.py
└── setup.cfg
```

## Setup

```bash
# Clone into your ROS 2 workspace
cd ~/lab08_ws/src
git clone git@github.com:Evan-Kusa/lab08_control.git

# Build
cd ~/lab08_ws
colcon build --packages-select lab08_control
source install/setup.bash
```

## Workflow

### 1. Start the Crazyflie driver

Make sure crazyswarm2 is running and connected to the drone via Crazyradio:

```bash
# (In a separate terminal, with crazyswarm2 workspace sourced)
ros2 launch crazyflie launch.py
```

### 2. Edit your PID gains

Open `config/pid_student.yaml` and enter your gains from Lab 07:

```yaml
goal_controller:
  ros__parameters:
    pid_x.kp: 2.5
    pid_x.ki: 0.0
    pid_x.kd: 0.05

    pid_y.kp: 2.5
    pid_y.ki: 0.0
    pid_y.kd: 0.05

    pid_z.kp: 2.5
    pid_z.ki: 0.01
    pid_z.kd: 0.05
```

### 3. Run the lab

```bash
# With your student gains:
ros2 launch lab08_control lab08.launch.py pid_file:=<path_to>/pid_student.yaml

# Or with the default (instructor) gains:
ros2 launch lab08_control lab08.launch.py
```

This launches three nodes:
- **goal_controller** — Reads your PID gains, subscribes to `/goal_pose`, publishes `/cmd_vel`
- **trajectory_publisher** — Sequences through the waypoints, logs actual vs. goal pose
- **plotter** — Independently records trajectory data

### 4. Collect results

When the course completes (or you press Ctrl+C), the following files are saved to the current directory:

| File | Description |
|------|-------------|
| `trajectory_log.csv` | Timestamped actual and goal positions |
| `traj_xyz_time.png` | X, Y, Z vs. time (actual vs. goal) |
| `traj_xy_path.png` | Top-down XY trajectory with waypoint markers |

Performance metrics are also printed to the terminal:
- Total run time
- RMS tracking error (overall and per-axis)
- Maximum tracking error

## Tuning PID gains at runtime

You can adjust gains without restarting using `ros2 param set`:

```bash
ros2 param set /goal_controller pid_x.kp 3.0
ros2 param set /goal_controller pid_x.kd 0.08
```

## Configuration

### Waypoints

Edit `config/waypoints.yaml` to change the course:

```yaml
waypoints:
  - [x, y, z, yaw]
  - [0.5, 0.0, 0.3, 0.0]
  ...
```

### PID output limits

The `min_output` / `max_output` values clamp the PID output velocity. Defaults:
- X/Y: [-0.707, 0.707] m/s
- Z: [-0.4, 0.4] m/s
- Yaw: [-1.0, 1.0] rad/s
