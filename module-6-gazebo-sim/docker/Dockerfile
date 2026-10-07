# ROS 2 Humble + Gazebo Fortress + a browser desktop (noVNC) + ROSA.
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

# Window rules: Gazebo opens maximized; the camera viewer opens large and on top
RUN mkdir -p /root/.config/openbox \
    && sed -e 's#<applications>#&<application name="ign-gazebo-gui"><maximized>yes</maximized></application>#' \
           -e 's#<applications>#&<application name="rqt_image_view"><position force="yes"><x>40</x><y>60</y></position><size><width>680</width><height>540</height></size><layer>above</layer></application>#' \
       /etc/xdg/openbox/rc.xml > /root/.config/openbox/rc.xml

# ROSA and the Ollama connector (same versions as the ROSA setup guide)
RUN pip3 install --no-cache-dir \
    jpl-rosa \
    'langchain-ollama<1.0' \
    'langchain-core>=0.3.52,<1.0' \
    pillow

# Start script for the browser desktop. sed removes Windows line endings.
COPY start-desktop.sh /usr/local/bin/start-desktop.sh
RUN sed -i 's/\r$//' /usr/local/bin/start-desktop.sh \
    && chmod +x /usr/local/bin/start-desktop.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
CMD ["/usr/local/bin/start-desktop.sh"]
