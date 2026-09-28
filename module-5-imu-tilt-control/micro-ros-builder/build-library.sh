#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_ROOT="${SCRIPT_DIR}/.build"
FIRMWARE_BUILD="${BUILD_ROOT}/firmware/build"
OUTPUT_DIR="${SCRIPT_DIR}/../pico2w_led/libmicroros"
STAGING_DIR="${OUTPUT_DIR}.tmp"
CHECK_DIR="${BUILD_ROOT}/archive-check"

set +u
source /opt/ros/humble/setup.bash
source /opt/micro_ros_ws/install/local_setup.bash
set -u

rm -rf "${BUILD_ROOT}" "${STAGING_DIR}"
mkdir -p "${BUILD_ROOT}"
cd "${BUILD_ROOT}"

apt-get update
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
ros2 run micro_ros_setup build_firmware.sh \
    "${SCRIPT_DIR}/toolchain.cmake" \
    "${SCRIPT_DIR}/colcon.meta"

ARCHIVE="${FIRMWARE_BUILD}/libmicroros.a"
if [[ ! -f "${ARCHIVE}" ]]; then
    echo "Expected archive was not produced: ${ARCHIVE}" >&2
    exit 1
fi

mkdir -p "${CHECK_DIR}"
cd "${CHECK_DIR}"
arm-none-eabi-ar x "${ARCHIVE}"
find . -type f \( -name '*.o' -o -name '*.obj' \) \
    -exec arm-none-eabi-readelf -A {} \; > attributes.txt 2>/dev/null

if grep -Eq 'v6-M|6-M|Cortex-M0' attributes.txt; then
    echo "Archive contains RP2040/Cortex-M0+ objects; refusing to publish it." >&2
    exit 1
fi

if ! grep -Eqi 'v8-M\.main(line)?|8-M\.MAIN|Cortex-M33' attributes.txt; then
    echo "Could not confirm ARMv8-M Mainline/Cortex-M33 objects." >&2
    exit 1
fi

mkdir -p "${STAGING_DIR}"
cp -R "${FIRMWARE_BUILD}/include" "${STAGING_DIR}/include"
cp "${ARCHIVE}" "${STAGING_DIR}/libmicroros.a"
cp attributes.txt "${STAGING_DIR}/architecture.txt"

rm -rf "${OUTPUT_DIR}"
mv "${STAGING_DIR}" "${OUTPUT_DIR}"

echo "Created ${OUTPUT_DIR}/libmicroros.a"
grep -E 'Tag_CPU_(name|arch)' "${OUTPUT_DIR}/architecture.txt" | sort -u