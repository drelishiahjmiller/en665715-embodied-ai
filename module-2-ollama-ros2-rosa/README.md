# Ollama + ROSA + ROS 2 Setup

## Architecture Overview

```
You (natural language) → ROSA + Ollama (on Mac) → ROS 2 (Docker) → WiFi → micro-ROS (Pico 2 W)
```

- **Ollama** runs natively on your Mac (LLM inference)
- **ROS 2 Humble** runs in Docker (robot middleware)
- **ROSA** is a Python library that translates natural language into ROS 2 commands via Ollama
- **Pico 2 W** (future) runs micro-ROS and communicates with ROS 2 over WiFi

## Prerequisites

- macOS with [Homebrew](https://brew.sh)
- Python 3.12 (`brew install python@3.12`) — 3.14 has compatibility issues with langchain
- [Ollama](https://ollama.com) installed and running locally
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (for ROS 2 + ROSA)

### Pull the Ollama model

```bash
ollama pull llama3.2
```

Verify Ollama is running:
```bash
curl http://localhost:11434/api/tags
```

## 1. Local Setup (Ollama only, no ROS 2)

This lets you test the Ollama Python library without ROS 2.

```bash
python3.12 -m venv venv
source venv/bin/activate
pip install ollama jpl-rosa 'langchain-ollama<1.0' 'langchain-core>=0.3.52,<1.0'
```

Test Ollama connection:
```bash
python test_ollama.py
```

Verify all imports:
```bash
python -c "from rosa import ROSA; from langchain_ollama import ChatOllama; from ollama import chat; print('All imports successful')"
```

### Activate/deactivate the venv

```bash
source venv/bin/activate   # activate (you'll see (venv) in your prompt)
deactivate                 # deactivate
which python               # verify — should show .../ollama/venv/bin/python
```

## 2. Docker Setup (ROS 2 + ROSA)

ROSA requires ROS 2 (`rclpy`) to function. The Docker container provides ROS 2 Humble with all Python dependencies pre-installed.

### First time: Build the image

```bash
docker compose up -d --build
```

> If `docker` is not found, add it to your PATH first:
> ```bash
> export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
> ```

### Start the container

```bash
docker compose up -d
```

The container runs in the background. You only need to do this once (or after a reboot / `docker compose down`).

### Run ROSA

```bash
docker compose exec ros2 bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/rosa_hello.py"
```

You can run this as many times as you want while the container is running.

### Enter the container interactively

```bash
docker compose exec ros2 bash
```

Once inside:
```bash
source /opt/ros/humble/setup.bash
python3 /workspace/rosa_hello.py
```

### Demo without hardware (mock ROS 2 topics)

Inside the container, you can simulate a ROS 2 environment:

```bash
# Terminal 1: Start turtlesim (built-in ROS 2 simulator)
ros2 run turtlesim turtlesim_node

# Terminal 2: Publish fake sensor data
ros2 topic pub /sensor_data std_msgs/String "data: 'IMU: roll=0.1, pitch=0.2'"

# Terminal 3: List active topics
ros2 topic list
```

Then use ROSA to interact with these topics via natural language.

### Stop the container

```bash
docker compose down
```

### Rebuild after Dockerfile changes

```bash
docker compose down
docker compose up -d --build
```

## File Overview

| File | Runs on | Purpose |
|---|---|---|
| `test_ollama.py` | Mac (venv) | Test Ollama connection without ROS 2 |
| `rosa_hello.py` | Docker | ROSA agent — natural language → ROS 2 commands |
| `Dockerfile` | — | Defines the ROS 2 container image |
| `docker-compose.yml` | — | Container configuration (networking, volumes) |

## Notes

- Inside Docker, Ollama is accessed via `host.docker.internal:11434` (already configured in `rosa_hello.py`)
- The `docker-compose.yml` mounts this folder into the container at `/workspace`, so you can edit files on your Mac and run them in Docker
- `network_mode: host` ensures the container can reach Ollama and (future) Pico 2 W over WiFi
- The container uses Python 3.10 (Ubuntu 22.04); the local venv uses Python 3.12

## Cross-Platform Support

This project works on **macOS, Linux, and Windows** — anywhere Docker and Ollama run.

### Platform differences

| | macOS | Windows | Linux |
|---|---|---|---|
| Docker Desktop | Required | Required | Docker Engine or Desktop |
| Ollama URL in Docker | `host.docker.internal:11434` | `host.docker.internal:11434` | `172.17.0.1:11434` (or add `--add-host=host.docker.internal:host-gateway` to docker-compose.yml) |
| PATH setup | May need `export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"` | Added automatically by installer | Added automatically |
| Local venv | `python3.12 -m venv venv` | `py -3.12 -m venv venv` | `python3.12 -m venv venv` |

### Linux users

If `host.docker.internal` doesn't resolve inside the container, add this to `docker-compose.yml` under the `ros2` service:

```yaml
extra_hosts:
  - "host.docker.internal:host-gateway"
```
