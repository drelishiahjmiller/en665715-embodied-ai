# Module 5: Pico 2 W IMU Tilt and Wheel Control

This is an isolated working copy of the Module 4 Pico 2 W micro-ROS project.
Make Module 5 firmware and ROSA changes here; do not edit
`../module-4-agent-architecture-mcp/`.

## Folders

| Folder | Purpose |
|---|---|
| `pico2w_led/` | Pico 2 W application, separated BNO055/motor/tilt modules, and the existing compatible micro-ROS archive |
| `micro-ros-builder/` | Copied builder sources; generated `.build/` output is not included |
| `module-5-pico-tilt-rosa/` | Copied Module 4 LED bridge and agent; add the Module 5 tilt tool and agent here |

The firmware keeps responsibilities separate:

| Source | Responsibility |
|---|---|
| `pico2w_led.c` | Wi-Fi, micro-ROS entities, ROS callbacks, and application loop |
| `bno055.c` / `bno055.h` | I2C setup, BNO055 initialization, pitch reading |
| `tb6612.c` / `tb6612.h` | TB6612FNG GPIO/PWM, direction, and stop |
| `tilt_control.c` / `tilt_control.h` | Tilt threshold, arm state, and five-second stop timer |

The Pico firmware's generated `build/` output is not copied. Build it from VS Code
inside `pico2w_led/`. `wifi_config.h` is a local configuration file; keep it private
and do not include it in screenshots or submissions.

## Use the course ROS 2 container

Run Compose from the parent `ollama/` folder. The existing Compose file bind-mounts
that folder at `/workspace`; this Module 5 copy does not add or replace a Compose
configuration.

```bash
docker compose up -d
docker compose exec ros2 bash -lc \
  "source /opt/ros/humble/setup.bash && \
   python3 /workspace/module-5-imu-tilt-control/module-5-pico-tilt-rosa/pico_tilt_agent.py"
```

Run the micro-ROS Agent using the same procedure as the Module 4 setup guide. Do
not run Module 4 and Module 5 Pico firmware at the same time with duplicate
`/pico2w_led` node names.
