"""Standalone ROSA agent for the Pico 2 W onboard LED."""

import sys

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

from pico_led_tool import set_pico_led


def build_agent() -> ROSA:
    llm = ChatOllama(
        model="llama3.2",
        temperature=0,
        base_url="http://host.docker.internal:11434",
    )

    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You control only the onboard LED of a Raspberry Pi Pico 2 W. "
            "Use set_pico_led for every request to turn the LED on or off. "
            "Report success only when the tool returns a matching acknowledgment. "
            "Report timeout or disconnection errors exactly and do not claim the LED changed."
        )
    )

    return ROSA(
        ros_version=2,
        llm=llm,
        tools=[set_pico_led],
        prompts=prompts,
        verbose=True,
    )


def main() -> None:
    prompt = " ".join(sys.argv[1:]).strip()
    if not prompt:
        prompt = "Turn on the Pico LED as a hello-world test."

    agent = build_agent()
    print(agent.invoke(prompt))


if __name__ == "__main__":
    main()