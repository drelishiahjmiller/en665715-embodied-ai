# Module 3: Agent Architecture + MCP

This module extends the Module 2 ROSA agent with tools loaded from standalone
Model Context Protocol (MCP) servers over stdio.

## Files

| File | Purpose |
|---|---|
| `rosa_agent_mcp.py` | Combines Module 2 robot tools with MCP tools |
| `mcp_datasheet_server.py` | Offline BNO055 and TB6612FNG datasheet lookup tools |
| `mcp_weather_server.py` | Live weather from Open-Meteo without an API key |

## Run

Run these commands from the repository root:

```bash
docker compose up -d --build
docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/module-3-agent-architecture-mcp/rosa_agent_mcp.py"
```

The MCP servers are launched automatically as subprocesses. They do not need
to be started separately.