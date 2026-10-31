# Module 9: Evaluation Metrics & Hypothesis Testing

## Lab: Distance from One Camera (Depth Estimation)

This lab estimates the distance to an object from a single ESP32-CAM photo with
[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) (metric, indoor models,
run on the CPU inside the ROS 2 container), calibrates the result with a tape measure, and lets a
ROSA agent drive to a ball and stop at a chosen distance. Students use it for Module 9's
evaluation and hypothesis-testing activities. It builds on Module 8: the same
simulation, sensor fusion, camera node, and ROSA movement tools, plus the `depth/` folder.

This folder is the finished reference project (the Module 8 project plus the depth lab). Students
build it themselves by adding the files to their own `module-8-sensor-fusion` project, following
the **Distance from One Camera** HTML guide supplied in the Module 9 course materials on Canvas.
The guide is maintained outside this code repository.

This is the **Module 9 depth-estimation discussion reference**, not the separate monocular
SLAM or Module 10 VLA experiments. Neither of those projects is needed to run this module.
Predicted depth is an estimate, not a depth-sensor measurement; calibrate and evaluate it
before supervised robot trials. It is not used as a safety-rated obstacle detector.

## Folders

| Folder | Purpose |
|---|---|
| `depth/` | The depth lab: `depth_estimator.py` (model, colored-object finder, calibration), `depth_node.py`, `depth_tools.py` (ROSA tools), `depth_agent.py`, `ask_depth.py`, `fit_depth.py`, `download_model.py`, `benchmark_depth.py` |
| `docker/` | Module 8's image plus CPU-only PyTorch, `transformers`, and `scipy`; models are saved in `depth/hf_cache` |
| `launch/` | `depth.launch.py` (new), `sensor_fusion.launch.py`, and `vision.launch.py` |
| `vision/` | Camera node (reused) and the Module 8 camera vision lab |
| `odometry/`, `sim/`, `config/` | Unchanged from Module 8 |

`firmware/` is not copied: the Pico (`pico_fusion`) and ESP32-CAM (`esp32cam_camera`) firmware from
Module 8 is used as it is.

## Topics

| Topic | Type | Publisher |
|---|---|---|
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `camera_node.py` |
| `/depth/ask`, `/depth/result` | `std_msgs/String` (JSON) | ROSA tools / `depth_node.py` |

## Run in Gazebo

```bash
cd module-9-depth-estimation
docker compose up --build -d
docker compose exec gazebo bash
python3 /workspace/depth/download_model.py     # one time: downloads the small model (99 MB)

ros2 launch /workspace/sim/launch/sim_vision.launch.py gui:=false                      # Terminal A
ros2 launch /workspace/launch/sensor_fusion.launch.py sim:=true                        # Terminal B
ros2 launch /workspace/launch/depth.launch.py sim:=true object_diameter_m:=0.14        # Terminal C
python3 /workspace/odometry/move_test.py turn 40                                       # Terminal D
python3 /workspace/depth/ask_depth.py "red ball" --true auto
python3 /workspace/depth/fit_depth.py --min 0.4 --max 2.5   # after 8+ measurements
python3 /workspace/depth/depth_agent.py
```

On the robot, start the depth launch with `camera_url:=http://CAMERA_IP/capture` instead of
`sim:=true`, and give `--true` the distance you measured with a tape.

Stop the Module 8 containers first: both projects use ports 6080 and 8888.

## Local downloads and results

The module's [`.gitignore`](.gitignore) excludes downloaded model weights in
`depth/hf_cache/`, measured calibration in `depth/calibration.json`, and generated
depth, camera, and odometry results. Each student downloads the model and collects their
own measurements locally; these files are not included in the GitHub discussion code.
