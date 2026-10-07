# Module 6: Gazebo Simulation and Sim-to-Real Control

This module adds a Gazebo Fortress obstacle-course simulation for the
differential-drive robot. ROSA can inspect its odometry, IMU, and camera,
plan collision-aware routes, and drive the simulated car using a local Ollama
model. The sim-to-real example sends the same bounded motion commands to the
Gazebo robot or the physical Pico 2 W car and compares their results.

## Folders

| Folder | Purpose |
|---|---|
| `agent/` | ROSA agent and safe Gazebo robot tools |
| `config/` | ROS 2 and Gazebo topic bridge configuration |
| `docker/` | ROS 2 Humble, Gazebo Fortress, ROSA, and browser desktop image |
| `launch/` | Main simulation launch file |
| `sim-to-real/` | Shared simulation/robot motion API, setup check, and Pico 2 W firmware |
| `urdf/` | Differential-drive robot, sensors, and Gazebo plugins |
| `worlds/` | Walled obstacle course and named goal |

## Run the Gazebo simulation

Ollama must be running on the host with the configured model available. The
default model is `llama3.2`; set `OLLAMA_MODEL` in the container environment to
use a different model.

From this module folder, build and start the container:

```bash
cd module-6-gazebo-sim
docker compose up --build -d
```

Open <http://localhost:6080/vnc.html> for the browser desktop. Then launch the
simulation:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch /workspace/launch/sim.launch.py"
```

Use `gui:=false` after the launch file path to run without the Gazebo window.

## Drive with ROSA

Keep the simulation running and start the agent in another terminal:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/agent/gazebo_car_agent.py"
```

The agent supports status checks, named or coordinate destinations, straight
drives, turns, emergency stops, and camera snapshots. Every movement requires
human approval and is constrained by speed, distance, and obstacle checks.

## Sim-to-real setup

Copy the Pico Wi-Fi configuration example, enter the local network and
micro-ROS Agent details, then build and flash the firmware with the included
Pico SDK VS Code project:

```bash
cd sim-to-real/pico2w_led
cp wifi_config.example.h wifi_config.h
```

Keep `wifi_config.h` private; it and generated build output are ignored by Git.
Start Gazebo with the micro-ROS Agent add-on:

```bash
cd module-6-gazebo-sim
docker compose \
  -f docker-compose.yml \
  -f sim-to-real/docker-compose.real-robot.yml \
  up --build -d
```

Launch the simulation with ground-truth position reporting:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch /workspace/sim-to-real/sim_with_ground_truth.launch.py"
```

After powering the Pico 2 W car, check both connections:

```bash
docker compose exec gazebo bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/sim-to-real/check_setup.py"
```

Pass `--wheel-test` only after lifting the real robot's wheels off the table.

## Stop the environment

```bash
docker compose down
```
