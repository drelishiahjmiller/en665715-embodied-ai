from langchain_ollama import ChatOllama
from rosa import ROSA

# 1. Configure the LLM — points to your local Ollama server
llm = ChatOllama(
    model="llama3.2",       # must match a model you've pulled
    temperature=0,            # deterministic output for reproducibility
    base_url="http://host.docker.internal:11434",  # Ollama on Mac host
)

# 2. Create the ROSA agent (no custom tools yet)
agent = ROSA(ros_version=2, llm=llm)

# 3. Send a natural language command
response = agent.invoke("List all active ROS 2 topics.")
print(response)