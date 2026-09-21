FROM ros:humble

SHELL ["/bin/bash", "-c"]

ARG MICRO_ROS_SETUP_COMMIT=99757720fdf33031a41288d8b69c7c0356f2db64
ARG PICO_SDK_COMMIT=079c6f39023649b154152db30f1d781e884879bc

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        gcc-arm-none-eabi \
        git \
        libnewlib-arm-none-eabi \
        libstdc++-arm-none-eabi-newlib \
        ninja-build \
        python3-colcon-common-extensions \
        pyflakes3 \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk \
    && git -C /opt/pico-sdk checkout "${PICO_SDK_COMMIT}" \
    && git -C /opt/pico-sdk submodule update --init --recursive

RUN mkdir -p /opt/micro_ros_ws/src \
    && git clone https://github.com/micro-ROS/micro_ros_setup.git /opt/micro_ros_ws/src/micro_ros_setup \
    && git -C /opt/micro_ros_ws/src/micro_ros_setup checkout "${MICRO_ROS_SETUP_COMMIT}" \
    && source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths /opt/micro_ros_ws/src --ignore-src -y --rosdistro humble \
    && cd /opt/micro_ros_ws \
    && colcon build --merge-install \
    && rm -rf build log /var/lib/apt/lists/*

ENV PICO_SDK_PATH=/opt/pico-sdk

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/micro_ros_ws/install/local_setup.bash' >> /root/.bashrc

WORKDIR /workspace/micro-ros-builder