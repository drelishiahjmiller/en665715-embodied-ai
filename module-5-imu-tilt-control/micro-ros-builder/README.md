# Pico 2 W micro-ROS Builder

This build-only Dev Container cross-compiles the ROS 2 Humble micro-ROS client
for the Pico 2 W ARM Cortex-M33 cores. It is separate from the `rosa-ros2`
runtime container.

Module 5 sets `RMW_UXRCE_MAX_PUBLISHERS=3` in `colcon.meta` for the LED state,
IMU pitch, and tilt status publishers. Rebuild the archive instead of reusing
the two-publisher Module 4 archive.

## Build

1. Open this folder in VS Code.
2. Run **Dev Containers: Reopen in Container**.
3. In the container terminal, run:

   ```bash
   bash build-library.sh
   ```

The script writes the verified archive and headers to:

```text
../pico2w_led/libmicroros/
├── architecture.txt
├── include/
└── libmicroros.a
```

The script rejects Cortex-M0+/ARMv6-M output. Wi-Fi credentials are not mounted
into the image during `docker build` and are not used by this builder.