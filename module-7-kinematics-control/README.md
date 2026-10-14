# Module 7: Kinematics, Odometry, and PID Control

This module adds wheel encoders, differential-drive kinematics, odometry-based
navigation, and closed-loop wheel-speed control to the course robot. The same
ROS 2 tools operate against a Gazebo stand-in or the physical Pico 2 W car, so
simulation measurements can be compared with real-world results.

## Folders

| Folder | Purpose |
|---|---|
| `firmware/pico_odometry/` | Pico 2 W encoder, motor, IMU, and micro-ROS firmware for odometry experiments |
| `firmware/pico_pid/` | Pico 2 W firmware with per-wheel PID speed control and telemetry |
| `odometry/` | Odometry node, motion tests, coordinate transforms, plots, and ROSA navigation agent |
| `pid/` | PID gain, step-response, telemetry, and ROSA driving tools |
| `sim/` | Gazebo robot, odometry arena, ros2_control configuration, and simulated Pico bridge |

## Start the environment

Ollama must be running on the host before using either ROSA agent. The default
model is `llama3.2`; set `OLLAMA_MODEL` in the container environment to use a
different local model.

From the Module 7 folder, build and start Gazebo and the micro-ROS Agent:

```bash
cd module-7-kinematics-control
docker compose up --build -d
```

Open <http://localhost:6080/vnc.html> for the browser desktop. The micro-ROS
Agent listens for the physical Pico 2 W on UDP port 8888.

## Run the odometry simulation

Start the Gazebo odometry arena:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch /workspace/sim/launch/sim_odometry.launch.py"
```

Use `gui:=false` after the launch file path to run without the Gazebo window.
The launch file starts a simulated Pico bridge that converts wheel joint angles
into the same `/pico/wheel_ticks` messages published by the real firmware.

In another terminal, start the wheel odometry node:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/odometry/odometry_node.py"
```

Run a known move to test the estimate:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/odometry/move_test.py square 1.0"
```

To record and plot all available position sources, start this command before a
test move and stop it afterward with Ctrl+C:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/odometry/odom_compare.py"
```

## Navigate with ROSA

Keep the simulation and odometry node running, then start the navigation agent:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/odometry/odom_agent.py"
```

The agent can report pose, drive a measured distance, turn by an angle, and
navigate to an `(x, y)` point. Each movement requires human approval.

## Run on the physical robot

Choose the firmware for the activity:

- `pico_odometry` publishes encoder counts and accepts velocity commands for
  odometry and navigation experiments.
- `pico_pid` additionally closes the wheel-speed loop, accepts runtime PID
  gains, and publishes controller telemetry.

Copy the Wi-Fi configuration example in the selected firmware folder and enter
the local network and micro-ROS Agent details:

```bash
cd firmware/pico_odometry
cp wifi_config.example.h wifi_config.h
```

For PID activities, use `firmware/pico_pid` instead. Keep `wifi_config.h`
private; credentials, micro-ROS libraries, and generated build output are
ignored by Git. Build and flash the selected firmware with its Pico SDK project.

With the physical robot powered and the Compose services running, use the same
odometry node and tools as the simulation. Do not run the Gazebo launch file at
the same time because its simulated Pico publishes the same topics.

## Tune and test PID control

Lift the physical robot's wheels before running a step-response test:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/pid/step_test.py"
```

Send gains without reflashing the Pico:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/pid/set_gains.py 1.0 5.0 0.0"
```

Start the PID-aware ROSA agent after the PID firmware is connected:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/pid/pid_agent.py"
```

The PID tools save CSV telemetry and PNG plots under `pid/results/`.

## Stop the environment

```bash
docker compose down
```
