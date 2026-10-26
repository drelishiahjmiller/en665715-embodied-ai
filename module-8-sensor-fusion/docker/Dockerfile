# Module 8: ROS 2 Humble + Gazebo Fortress + browser desktop + ROSA,
# ros2_control (diff_drive_controller), matplotlib, and robot_localization (EKF).
# The ros:humble image is multi-architecture, so this file works on
# Apple Silicon Macs, Intel Macs, and Windows PCs without changes.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Fortress and the ROS 2 <-> Gazebo packages, plus robot model tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-rqt-image-view \
    ros-humble-teleop-twist-keyboard \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Browser desktop: a virtual screen, a VNC server, noVNC, and a small window manager
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    python3-websockify \
    openbox \
    xterm \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Window rule: Gazebo opens maximized
RUN mkdir -p /root/.config/openbox \
    && sed -e 's#<applications>#&<application name="ign-gazebo-gui"><maximized>yes</maximized></application>#' \
       /etc/xdg/openbox/rc.xml > /root/.config/openbox/rc.xml

# NEW in Module 7: ros2_control with diff_drive_controller
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gz-ros2-control \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2controlcli \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Fortress looks for system plugins here (needed by gz_ros2_control)
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib

# ROSA and the Ollama connector (same versions as the ROSA setup guide),
# plus matplotlib for plots (from pip, so it matches ROSA's NumPy version)
RUN pip3 install --no-cache-dir \
    jpl-rosa \
    'langchain-ollama<1.0' \
    'langchain-core>=0.3.52,<1.0' \
    'matplotlib>=3.8'

# NEW in Module 8: robot_localization, the EKF that fuses wheels and IMU
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Start script for the browser desktop. sed removes Windows line endings.
COPY start-desktop.sh /usr/local/bin/start-desktop.sh
RUN sed -i 's/\r$//' /usr/local/bin/start-desktop.sh \
    && chmod +x /usr/local/bin/start-desktop.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
CMD ["/usr/local/bin/start-desktop.sh"]
