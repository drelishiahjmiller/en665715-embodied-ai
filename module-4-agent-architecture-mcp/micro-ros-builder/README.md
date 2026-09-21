# Pico 2 W micro-ROS Builder

This build-only Dev Container cross-compiles the ROS 2 Humble micro-ROS client
for the Pico 2 W ARM Cortex-M33 cores. It is separate from the `rosa-ros2`
runtime container.

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