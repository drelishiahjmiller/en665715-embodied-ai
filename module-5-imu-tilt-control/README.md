# Module 5: Pico 2 W IMU Tilt and Wheel Control

This module extends the Module 4 Pico 2 W micro-ROS project with a BNO055 IMU,
a TB6612FNG motor driver, and a ROSA agent for tilt-triggered wheel control.
The agent arms a forward or backward movement request, and the Pico executes
that request after detecting a sustained physical tilt.

## Behavior

1. Ask the ROSA agent to move forward or backward.
2. The agent publishes the request and waits for the Pico to acknowledge that
   it is armed.
3. The Pico records its current pitch as the reference position.
4. Tilting the Pico at least 2 degrees for 100 milliseconds starts both wheels
   in the requested direction.
5. The wheels run at full configured PWM for 10 seconds and then stop locally.

The requested movement direction is independent of the tilt direction. The
Pico also rejects invalid or duplicate requests and reports connection,
timeout, busy-state, and IMU errors to the agent.

## Folders

| Folder | Purpose |
|---|---|
| `pico2w_led/` | Pico 2 W firmware, BNO055 support, motor driver, tilt state machine, and micro-ROS transport |
| `micro-ros-builder/` | Reproducible micro-ROS static library build tooling |
| `module-5-pico-tilt-rosa/` | ROSA agents and ROS 2 tools for LED and tilt control |

The firmware keeps responsibilities separate:

| Source | Responsibility |
|---|---|
| `pico2w_led.c` | Wi-Fi, micro-ROS entities, ROS callbacks, and application loop |
| `bno055.c` / `bno055.h` | I2C setup, BNO055 initialization, pitch reading |
| `tb6612.c` / `tb6612.h` | TB6612FNG GPIO/PWM, direction, and stop |
| `tilt_control.c` / `tilt_control.h` | Tilt trigger, movement state, and 10-second stop timer |

## ROS 2 Topics

| Topic | Type | Purpose |
|---|---|---|
| `/pico/tilt/request` | `std_msgs/msg/Int32` | Arm forward (`1`) or backward (`-1`) movement |
| `/pico/tilt/status` | `std_msgs/msg/Int32` | Report ready, running, stopped, error, or busy states |
| `/pico/pitch` | `std_msgs/msg/Float32` | Publish the current BNO055 pitch |

## Configure and build the Pico firmware

Copy the Wi-Fi configuration example and enter the local network and
micro-ROS Agent details:

```bash
cd module-5-imu-tilt-control/pico2w_led
cp wifi_config.example.h wifi_config.h
```

Keep `wifi_config.h` private; it is ignored by Git. Build and flash the firmware
from VS Code using the included Pico SDK tasks. Generated `build/` output is not
committed.

## Use the course ROS 2 container

Run Compose from the repository root. The Compose configuration bind-mounts the
repository at `/workspace`.

```bash
docker compose up -d
docker compose exec ros2 bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/module-5-imu-tilt-control/module-5-pico-tilt-rosa/pico_tilt_agent.py"
```

Run the micro-ROS Agent using the same procedure as the Module 4 setup guide. Do
not run Module 4 and Module 5 Pico firmware at the same time with duplicate
`/pico2w_led` node names.
