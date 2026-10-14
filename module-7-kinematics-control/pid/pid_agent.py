"""ROSA agent that drives the robot through the Pico's PID speed controller.

Run inside the container with the PID firmware running:
    python3 /workspace/pid/pid_agent.py

Example requests:
    Drive forward at 0.15 meters per second for 3 seconds.
    Drive at 0.2 m/s while turning left at 0.5 rad/s for 4 seconds.
    Drive at 0.1 m/s turning right at 1 rad/s for 3 seconds.
    What are the wheel speeds?
"""

import os

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

import pid_tools

OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")


def build_agent():
    llm = ChatOllama(model=OLLAMA_MODEL, temperature=0, num_ctx=16384, base_url=OLLAMA_URL)
    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You drive a small two-wheel robot. Its Pico runs a PID controller that makes "
            "each wheel reach the commanded speed."),
        about_your_capabilities=(
            "drive_straight(direction, speed_mps, seconds) drives forward or backward. "
            "drive_arc(turn, speed_mps, turn_rate_radps, seconds) drives forward while turning "
            "left or right. spin(turn, turn_rate_radps, seconds) turns in place. Each reports "
            "how well the wheels tracked their target speeds. wheel_speeds() reports the "
            "current wheel speeds. stop_robot() stops the robot."),
        critical_instructions=(
            "For each request, call exactly one robot tool once. Copy every number exactly as "
            "the user wrote it, for example '0.15'. The tool result is shown to the human "
            "directly. Never claim the robot moved unless the tool reported it."),
    )
    return ROSA(ros_version=2, llm=llm,
                tools=[pid_tools.drive_straight, pid_tools.drive_arc, pid_tools.spin,
                       pid_tools.wheel_speeds, pid_tools.stop_robot],
                prompts=prompts, verbose=True, streaming=False,
                accumulate_chat_history=False, max_iterations=6)


def main():
    pid_tools.get_robot()
    agent = build_agent()
    print(f"PID robot agent ready (model: {OLLAMA_MODEL}). Type 'quit' to exit.")
    try:
        while True:
            try:
                request = input("You> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if request.lower() in ("quit", "exit"):
                break
            if request:
                try:
                    print(f"ROSA> {agent.invoke(request)}")
                except KeyboardInterrupt:
                    pid_tools.get_robot().stop()
                    print("\nInterrupted. The robot was stopped.")
    finally:
        pid_tools.close_robot()


if __name__ == "__main__":
    main()
