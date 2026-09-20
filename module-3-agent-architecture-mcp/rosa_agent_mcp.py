"""Module 3 example: combine local ROSA tools with tools from MCP servers.

Run inside the ROS 2 Docker container from the repository root:
    docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/module-3-agent-architecture-mcp/rosa_agent_mcp.py"
"""
import asyncio
import sys
from pathlib import Path

from langchain_ollama import ChatOllama
from langchain_mcp_adapters.client import MultiServerMCPClient
from rosa import ROSA, RobotSystemPrompts

MODULE_DIR = Path(__file__).resolve().parent
MODULE_2_DIR = MODULE_DIR.parent / "module-2-ollama-ros2-rosa"
sys.path.insert(0, str(MODULE_2_DIR))

from robot_tools import (  # noqa: E402
    read_imu,
    move_forward,
    read_tof_distance,
    rotate,
    emergency_stop,
    get_robot_status,
)

LOCAL_TOOLS = [read_imu, move_forward, read_tof_distance, rotate, emergency_stop, get_robot_status]

MCP_SERVERS = {
    "datasheets": {
        "command": "python3",
        "args": [str(MODULE_DIR / "mcp_datasheet_server.py")],
        "transport": "stdio",
    },
    "weather": {
        "command": "python3",
        "args": [str(MODULE_DIR / "mcp_weather_server.py")],
        "transport": "stdio",
    },
}


async def build_agent() -> ROSA:
    llm = ChatOllama(
        model="llama3.2",
        temperature=0,
        base_url="http://host.docker.internal:11434",
    )

    mcp_client = MultiServerMCPClient(MCP_SERVERS)
    mcp_tools = await mcp_client.get_tools()
    print(f"Loaded {len(mcp_tools)} MCP tool(s): {[tool.name for tool in mcp_tools]}")

    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You are a differential-drive robotic car equipped with a BNO055 IMU, "
            "VL53L0X Time-of-Flight distance sensor, quadrature wheel encoders, "
            "and an ESP32-CAM. You are controlled via ROS 2 on a Raspberry Pi Pico 2W. "
            "You also have offline datasheet lookup tools (search_bno055_docs, "
            "search_tb6612fng_docs) for answering hardware questions, and a "
            "get_current_weather tool for checking conditions before outdoor deployment. "
            "Always check for obstacles before moving. Never exceed 5.0 m/s. "
            "If you are unsure about safety, call emergency_stop."
        )
    )

    return ROSA(
        ros_version=2,
        llm=llm,
        tools=LOCAL_TOOLS + mcp_tools,
        prompts=prompts,
        verbose=True,
    )


async def main():
    agent = await build_agent()

    print("\n=== Test 1: Local robot tool ===")
    print(agent.invoke("What is the current IMU heading?"))

    print("\n=== Test 2: External MCP tool ===")
    print(agent.invoke("What I2C address does the BNO055 use, according to the datasheet notes?"))

    print("\n=== Test 3: Mixed local + MCP tools ===")
    print(agent.invoke(
        "Check the TB6612FNG direction control notes, then move forward 1 meter and report the new heading."
    ))

    print("\n=== Test 4: Weather MCP tool (Baltimore, MD) ===")
    print(agent.invoke(
        "What is the current weather at latitude 39.29, longitude -76.61? "
        "Is it safe to deploy the robot outdoors right now?"
    ))


if __name__ == "__main__":
    asyncio.run(main())