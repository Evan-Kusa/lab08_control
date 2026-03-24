# Lab 08 Control — Crazyflie Real Drone

ROS 2 (Jazzy) package for Lab 08: flying a real Crazyflie 2.1 through a waypoint course using student-tuned PID gains from Lab 07 (simulation).

## Prerequisites

- Docker (recommended) **or** ROS 2 Jazzy installed natively
- [crazyswarm2](https://github.com/IMRCLab/crazyswarm2) installed and configured
- Crazyradio 2.0 USB dongle (or Crazyradio PA)
- Crazyflie 2.1 with Lighthouse deck and Flow deck v2
- Two Lighthouse V2 base stations (channels 1 and 2)
- Python packages: `numpy`, `matplotlib`, `pyyaml`, `tf_transformations`

## Hardware Notes

### Drone Addressing

Each Crazyflie has a radio address in the format `E7E7E7E7D#`, where `#` is the single-digit number written on the drone. For example:

| Drone Label | Radio Address |
|-------------|---------------|
| 0 | `radio://0/80/2M/E7E7E7E7D0` |
| 2 | `radio://0/80/2M/E7E7E7E7D2` |
| 3 | `radio://0/80/2M/E7E7E7E7D3` |

**Important:** The default Crazyflie address `E7E7E7E7E7` is **not** used. Update the URI in `crazyflie_real_crazyswarm2.yaml` (in the crazyswarm2 config) to match the drone you are flying.

### Crazyradio 2.0

The lab uses a Crazyradio 2.0 (USB product ID `1915:7777`). cflib 0.1.31+ supports it. The Docker container must have `/dev/bus/usb` mounted.

### Lighthouse Setup

- Base stations are Lighthouse V2 on channels 1 and 2.
- **Geometry does not persist across power cycles.** After every battery swap, you must restore the calibration data. Use `reset_drone.py` or `restore_lighthouse_geo.py` from the `crazyflie/` utility scripts.
- cfclient's 2-step geometry estimation produces correct relative geometry but can produce a rotated world frame. Use `fix_lighthouse_frame.py` if needed.

### TF Frames

The crazyswarm2 driver publishes TF frames based on the robot name in the YAML config. With a robot named `crazyflie_real`:

| Frame | Value |
|-------|-------|
| World frame | `crazyflie_real/odom` |
| Robot frame | `crazyflie_real` |

These are the defaults in this package. If your crazyswarm2 config uses a different name (e.g., `cf231`), update via launch arguments:

```bash
ros2 launch lab08_control lab08.launch.py robot_base_frame:=cf231 robot_world_frame:=cf231/odom
```

### Control Interface

This package publishes `Hover` messages directly to `/crazyflie_real/cmd_hover`, **bypassing vel_mux entirely**. This is intentional — the cflib backend does not subscribe to `cmd_velocity_world`, and vel_mux interprets negative `linear.z` as a land command, causing repeated drop-fly cycles.

Because vel_mux is bypassed, **takeoff must be triggered manually** before starting the trajectory:

```bash
ros2 service call /crazyflie_real/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.3, duration: {sec: 2, nanosec: 0}}"
```

Wait ~3 seconds, then launch `lab08`.

Z height is controlled via the `z_distance` field in the Hover message (a setpoint, not a velocity), so the Z PID is not used at runtime.

Landing is handled automatically — when all waypoints are completed, the trajectory publisher calls the `/crazyflie_real/land` service.

## Quick Start (Docker)

```bash
# 1. Build the base image (one-time, ~15-20 min)
cd ros2-docker-images/base_images
docker build -f Dockerfile.base-core -t base-core:latest .

# 2. Build the Crazyflie real-drone image (one-time, ~5-10 min)
cd ros2-docker-images/crazyflie
docker build -f Dockerfile.crazyflie-real -t crazyflie-real:latest .

# 3. Run the container (plug in Crazyradio first)
docker run -it --rm --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /path/to/crazyflie/scripts:/root/scripts \
    -v /path/to/lab08_control:/opt/lab08_ws/src/lab08_control \
    crazyflie-real:latest
```

Inside the container:
- `cf-scan` — Scan for nearby Crazyflies
- `lab08` — Launch with default (instructor) PID gains
- `lab08-student` — Launch with student PID gains

## Launch Sequence

```bash
# 1. Start the container (see above)

# 2. Start the crazyflie driver (in tmux pane 1)
ros2 launch crazyflie launch.py mocap:=False

# 3. Restore lighthouse geometry (after any battery swap)
python3 /root/scripts/restore_lighthouse_geo.py

# 4. Takeoff
ros2 service call /crazyflie_real/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.3, duration: {sec: 2, nanosec: 0}}"

# 5. Wait ~3s, then run lab08 (in tmux pane 2)
lab08
# Or with a specific student's gains:
ros2 launch lab08_control lab08.launch.py student:=3

# After a crash (no battery swap):
python3 /root/scripts/reset_drone.py
# Then repeat steps 2-5

# After a battery swap:
# Repeat steps 2-5 (step 3 is critical — geometry doesn't persist)
```

## Package Structure

```
lab08_control/
├── config/
│   ├── pid_defaults.yaml          # Known-good PID gains (instructor reference)
│   ├── pid_student.yaml           # Template for student gains
│   ├── student_gains_example.yaml # Template for per-student gains file
│   └── waypoints.yaml            # Lab course waypoints
├── launch/
│   └── lab08.launch.py           # Launches everything with one command
├── lab08_control/
│   ├── goal_controller.py        # PID controller (publishes Hover to cmd_hover)
│   ├── trajectory_publisher.py   # Waypoint sequencer + data logger + land service
│   ├── plotter.py                # Standalone trajectory plotter
│   └── control_services.py       # Velocity mux (legacy, not used with Hover)
├── package.xml
├── setup.py
└── setup.cfg
```

## Student Workflow

### 1. Edit PID gains

Either edit `config/pid_student.yaml` directly, or create a `config/student_gains.yaml` from the example template with per-student entries (see `student_gains_example.yaml`).

### 2. Run the lab

```bash
# With individual student gains (from student_gains.yaml):
ros2 launch lab08_control lab08.launch.py student:=3

# With a custom PID file:
ros2 launch lab08_control lab08.launch.py pid_file:=<path_to>/pid_student.yaml

# With default (instructor) gains:
ros2 launch lab08_control lab08.launch.py
```

### 3. Collect results

When the course completes, the drone lands automatically and the following files are saved:

| File | Description |
|------|-------------|
| `trajectory_log.csv` (or `trajectory_log_student3.csv`) | Timestamped actual and goal positions |
| `traj_xyz_time.png` (or `traj_student3_xyz_time.png`) | X, Y, Z vs. time (actual vs. goal) |
| `traj_xy_path.png` (or `traj_student3_xy_path.png`) | Top-down XY trajectory with waypoint markers |

Performance metrics are printed to the terminal:
- Total run time
- RMS tracking error (overall and per-axis)
- Maximum tracking error

## Tuning PID gains at runtime

You can adjust gains without restarting:

```bash
ros2 param set /goal_controller pid_x.kp 3.0
ros2 param set /goal_controller pid_x.kd 0.08
```

## Configuration

### Waypoints

Edit `config/waypoints.yaml` to change the course. The last waypoint should be the pre-landing position — landing is handled automatically via the land service.

```yaml
waypoints:
  - [x, y, z, yaw]
  - [0.5, 0.0, 0.3, 0.0]
  ...
```

### PID output limits

The `min_output` / `max_output` values clamp the PID output velocity. Defaults:
- X/Y: [-0.707, 0.707] m/s
- Z: [-0.4, 0.4] m/s (not used with Hover z_distance)
- Yaw: [-1.0, 1.0] rad/s

## Known Issues

- Drone 0 (address `E7E7E7E7D0`) did not respond to radio scan during Spring 2026 testing. Drone 2 (`E7E7E7E7D2`) works.
- `cflinkcpp` (Crazyradio 2.0 native library) is in alpha. The pip package fails to build due to pybind11 CMake version; must build from source with a sed fix (handled in the Dockerfile).
- The crazyswarm2 GUI node requires `rowan` and `nicegui` which are not declared as pip dependencies in the ROS package.
- Lighthouse geometry does not persist across power cycles. Always run `reset_drone.py` or `restore_lighthouse_geo.py` after a battery swap.
