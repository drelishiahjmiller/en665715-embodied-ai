from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

# Import your custom tools
from robot_tools import (
    read_imu,
    move_forward,
    read_tof_distance,
    rotate,
    emergency_stop,
    get_robot_status,
)

# ── 1. Configure the LLM ──────────────────────────────────────────
llm = ChatOllama(
    model="llama3.2",
    temperature=0,
    base_url="http://host.docker.internal:11434",  # Ollama on host
)

# ── 2. Define system prompts ──────────────────────────────────────
prompts = RobotSystemPrompts(
    embodiment_and_persona=(
        "You are a differential-drive robotic car equipped with a BNO055 IMU, "
        "VL53L0X Time-of-Flight distance sensor, quadrature wheel encoders, "
        "and an ESP32-CAM. You are controlled via ROS 2 on a Raspberry Pi Pico 2W. "
        "Always check for obstacles before moving. Never exceed 5.0 m/s. "
        "If you are unsure about safety, call emergency_stop."
    )
)

# ── 3. Create the ROSA agent with tools ──────────────────────────
agent = ROSA(
    ros_version=2,
    llm=llm,
    tools=[
        read_imu,
        move_forward,
        read_tof_distance,
        rotate,
        emergency_stop,
        get_robot_status,
    ],
    prompts=prompts,
    verbose=True,
)

# ── 4. Send commands ─────────────────────────────────────────────
print("=== Test 1: Read sensor data ===")
response = agent.invoke("What is the current IMU heading?")
print(response)

print("\n=== Test 2: Multi-step task ===")
response = agent.invoke(
    "Check for obstacles, then move forward 1 meter and report the new heading."
)
print(response)