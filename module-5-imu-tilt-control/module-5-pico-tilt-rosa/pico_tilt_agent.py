from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

from pico_tilt_tool import arm_tilt_move


def build_agent() -> ROSA:
    llm = ChatOllama(
        model="llama3.2",
        temperature=0,
        # ROSA's prompt and tool list are about 5,000 tokens; Ollama's default
        # 4,096-token context would silently cut off the tool definitions.
        num_ctx=16384,
        base_url="http://host.docker.internal:11434",
    )
    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You control one Pico 2 W wheeled robot. For every requested wheel "
            "movement, call arm_tilt_move with exactly 'forward' or 'backward'. "
            "Do not invent other directions or call other tools. After the Pico "
            "is armed, any small tilt starts both wheels in the requested "
            "direction at full speed for ten seconds before they stop locally. "
            "Repeat the tool result "
            "exactly. If it reports an error, tell the user not to tilt and do "
            "not claim that the robot is ready or has moved."
        )
    )
    return ROSA(
        ros_version=2,
        llm=llm,
        tools=[arm_tilt_move],
        prompts=prompts,
        verbose=True,
    )


def main() -> None:
    agent = build_agent()
    print("Pico tilt controller ready. Type 'quit' to exit.")
    while True:
        try:
            request = input("You> ").strip()
        except EOFError:
            print()
            break
        if request.lower() in ("quit", "exit"):
            break
        if request:
            print(f"ROSA> {agent.invoke(request)}")


if __name__ == "__main__":
    main()
