"""Ask the camera how far away something is (no robot movement).

Run inside the container while launch/depth.launch.py is running:
    python3 /workspace/depth/ask_depth.py                         # distances ahead, left, right
    python3 /workspace/depth/ask_depth.py "red ball"              # distance to the red ball
    python3 /workspace/depth/ask_depth.py "red ball" --true 0.75  # also record the distance
                                                                  # you measured with a tape (m)
    python3 /workspace/depth/ask_depth.py "red ball" --true auto  # Gazebo only: use the
                                                                  # simulator's true distance

Prints the answer and how long it took. The photo and its depth map are saved in
/workspace/depth/results/images/ and the answer is added to depth_log.csv, which
fit_depth.py uses to calibrate the camera.
"""

import argparse
import math

import rclpy

import depth_tools
import odom_tools   # depth_tools puts the odometry folder on the import path

# Gazebo vision arena: the red ball's center and radius (m), and the camera's
# distance in front of the robot's base_link (from course_robot.urdf.xacro)
RED_BALL = (1.4, 1.1, 0.07)
CAMERA_FORWARD = 0.06


def gazebo_true_distance():
    """Distance from the simulated camera to the nearest surface of the red ball."""
    robot = odom_tools.get_robot()
    if not robot.ready():
        raise SystemExit("--true auto needs the sensor fusion launch (and Gazebo) running.")
    pose = robot.pose("ground_truth")
    if pose is None:
        raise SystemExit("--true auto works only in Gazebo (no ground-truth pose found).")
    x, y, heading = pose
    cx, cy = x + CAMERA_FORWARD * math.cos(heading), y + CAMERA_FORWARD * math.sin(heading)
    return math.hypot(RED_BALL[0] - cx, RED_BALL[1] - cy) - RED_BALL[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("target", nargs="*", help="color and name of an object, e.g. red ball")
    parser.add_argument("--true", dest="true_m", default=None,
                        help="the real distance in meters (or 'auto' in Gazebo) to record")
    args = parser.parse_args()
    target = " ".join(args.target).strip()
    true_m = None
    if args.true_m == "auto":
        true_m = round(gazebo_true_distance(), 3)
    elif args.true_m is not None:
        true_m = float(args.true_m)

    client = depth_tools.get_client()
    print(f"Measuring: {target or 'the distances ahead'} ...")
    answer = client.ask(target, true_m)
    if answer.get("error"):
        print(f"Error: {answer['error']}")
    else:
        if target:
            if answer["target_found"]:
                print(f"  distance to {target:<9}: {answer['target_m']:.2f} m   "
                      f"(model said {answer['target_raw_m']:.2f} m; "
                      f"calibration {answer['scale']:.3f} x d^{answer['power']:.2f})")
                print(f"  bearing              : {depth_tools.side(answer['bearing_deg'])}")
                if "target_size_m" in answer:
                    print(f"  from the target size : {answer['target_size_m']:.2f} m "
                          "(pinhole camera, no depth model)")
                if true_m is not None:
                    error = 100 * (answer["target_m"] - true_m) / true_m
                    print(f"  recorded real distance: {true_m:.2f} m   error {error:+.0f} %")
            else:
                print(f"  {target}: not found in the photo")
        print(f"  straight ahead       : {answer['ahead_m']:.2f} m")
        print(f"  left / right         : {answer['left_m']:.2f} m / {answer['right_m']:.2f} m")
        print(f"  model / time         : {answer['model']}, {answer['seconds']:.1f} s")
        print(f"  photo                : /workspace/{answer['image']}")
        print(f"  photo + depth map    : /workspace/{answer['depth_image']}")
    depth_tools.close_client()
    if args.true_m == "auto":
        odom_tools.close_robot()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
