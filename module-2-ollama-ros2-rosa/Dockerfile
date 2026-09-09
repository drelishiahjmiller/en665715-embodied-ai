FROM --platform=linux/arm64 arm64v8/ros:humble

# Install Python pip and ROS 2 Python client library
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    ollama \
    jpl-rosa \
    'langchain-ollama<1.0' \
    'langchain-core>=0.3.52,<1.0'

# Source ROS 2 automatically in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH}"
ENV LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH}"
ENV AMENT_PREFIX_PATH="/opt/ros/humble"

WORKDIR /workspace
