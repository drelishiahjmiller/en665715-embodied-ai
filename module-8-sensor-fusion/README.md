# Module 8: Multi-Sensor Fusion and State Estimation

This module fuses the course robot's wheel odometry with its BNO055 IMU using the
ROS 2 `robot_localization` Extended Kalman Filter (EKF). The Pico firmware now
publishes the BNO055's full output (quaternion orientation, gyroscope, and
linear acceleration) as a timestamped `sensor_msgs/Imu` at 50 Hz. The same
fusion launch file runs against the Gazebo stand-in and the physical robot.

Lab guide 2 adds an ESP32-CAM: a camera node publishes its photos, a vision node
asks an Ollama vision model (`gemma4`) structured questions about them, and a
ROSA agent (planning with `llama3.2`) uses the answers to find and approach
objects with the fused position estimate.

The step-by-step lab guides are in `Module8_Multi_Sensor_Fusion_and_State_Estimation/`:
`multi-sensor-fusion-ekf-guide.html` (lab 1) and `esp32cam-vision-agent-guide.html` (lab 2).

## Folders

| Folder | Purpose |
|---|---|
| `config/` | `ekf.yaml`: which wheel and IMU values the EKF fuses |
| `launch/` | `sensor_fusion.launch.py`: wheel odometry, IMU mount transform, and the EKF |
| `firmware/pico_fusion/` | Pico 2 W firmware: encoders, motors, and full BNO055 data over micro-ROS |
| `firmware/esp32cam_camera/` | Lab 2: ESP32-CAM photo server sketch (VS Code + PlatformIO via `platformio.ini`, or Arduino IDE; AI Thinker ESP32-CAM) |
| `vision/` | Lab 2: camera node, vision node, ROSA camera and mission tools, agent, `ask_camera.py` |
| `odometry/` | Wheel odometry node, motion tests, fused-vs-wheel comparison, and ROSA agent |
| `sim/` | Gazebo robot with IMU, test arena, and the simulated Pico bridge |

## Topics

| Topic | Type | Publisher |
|---|---|---|
| `/pico/wheel_ticks` | `std_msgs/Int32MultiArray` | Pico (or `sim_pico_bridge.py`) |
| `/pico/imu/data` | `sensor_msgs/Imu` | Pico (or `sim_pico_bridge.py`) |
| `/pico/imu/std` | `std_msgs/Float32MultiArray` | You: live IMU uncertainty tuning |
| `/wheel/odom` | `nav_msgs/Odometry` | `odometry_node.py` (wheels only, no TF) |
| `/odometry/filtered` | `nav_msgs/Odometry` | `ekf_filter_node` (fused estimate) |
| `/tf` `odom -> base_link` | transform | `ekf_filter_node` only |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `camera_node.py` (lab 2) |
| `/vision/ask`, `/vision/result` | `std_msgs/String` (JSON) | ROSA tools / `vision_node.py` (lab 2) |

## Run in Gazebo

```bash
cd module-8-sensor-fusion
docker compose up --build -d
docker compose exec gazebo bash
# Terminal A
ros2 launch /workspace/sim/launch/sim_odometry.launch.py
# Terminal B
ros2 launch /workspace/launch/sensor_fusion.launch.py sim:=true
# Terminal C
python3 /workspace/odometry/move_test.py square 1.0
```

Add `left_wheel_error:=0.01` to the simulation launch to give the left wheel a
1 % encoder error and see the IMU correct the heading.

## Run on the robot

Flash `firmware/pico_fusion` (build with the Raspberry Pi Pico VS Code
extension), keep the robot still until the serial console prints
`Gyroscope calibrated`, then:

```bash
ros2 launch /workspace/launch/sensor_fusion.launch.py \
  wheel_diameter:=YOUR_D wheel_separation:=YOUR_L
```

## Lab 2: camera vision

```bash
ros2 launch /workspace/sim/launch/sim_vision.launch.py                 # Terminal A
ros2 launch /workspace/launch/sensor_fusion.launch.py sim:=true        # Terminal B
ros2 launch /workspace/launch/vision.launch.py sim:=true               # Terminal C
python3 /workspace/vision/vision_agent.py                              # Terminal D
```

On the robot, flash `firmware/esp32cam_camera` and start the vision launch with
`camera_url:=http://CAMERA_IP/capture` instead of `sim:=true`.

`wifi_config.h` files and `libmicroros/` are local files and are not part of the
published source.
