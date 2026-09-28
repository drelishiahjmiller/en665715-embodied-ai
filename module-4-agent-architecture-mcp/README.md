# Module 4: Agent Architecture + MCP + Pico 2 W

This module extends the Module 2 ROSA agent with tools loaded from standalone
Model Context Protocol (MCP) servers over stdio. It also includes a Pico 2 W
micro-ROS LED example and a ROSA tool that controls the LED through ROS 2.

## Files

| File | Purpose |
|---|---|
| `rosa_agent_mcp.py` | Combines Module 2 robot tools with MCP tools |
| `mcp_datasheet_server.py` | Offline BNO055 and TB6612FNG datasheet lookup tools |
| `mcp_weather_server.py` | Live weather from Open-Meteo without an API key |
| `memory_manager.py` | Short-term window and summary memory |
| `mcp_memory_server.py` | Persistent semantic and episodic memory tools |
| `rosa_agent_mcp_memory.py` | Interactive ROSA agent combining tools and memory |
| `module-4-pico-led-rosa/` | ROSA agent and ROS 2 tool for Pico LED control |
| `pico2w_led/` | Pico 2 W micro-ROS firmware source |
| `micro-ros-builder/` | Reproducible micro-ROS library build tooling |

## Run

Run these commands from the repository root:

```bash
docker compose up -d --build
docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/module-4-agent-architecture-mcp/rosa_agent_mcp.py"
```

The MCP servers are launched automatically as subprocesses. They do not need
to be started separately.

## Run the Memory-Aware Agent

Install the embedding model on the host once:

```bash
ollama pull nomic-embed-text
```

Then rebuild and run the separate memory-enabled agent from this directory's
parent (`ollama`):

```bash
docker compose up -d --build
docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/module-4-agent-architecture-mcp/rosa_agent_mcp_memory.py"
```

Semantic and episodic memories persist in `memory_db/`. The original
`rosa_agent_mcp.py` remains the baseline MCP-only activity.