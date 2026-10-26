"""Ask the camera one question from the command line (no robot movement).

Run inside the container while launch/vision.launch.py is running:
    python3 /workspace/vision/ask_camera.py                  # describe the scene
    python3 /workspace/vision/ask_camera.py "red ball"       # look for an object

Prints the vision model's answer and how long it took. The photo is saved in
/workspace/vision/results/images/ and the answer is added to vision_log.csv.
"""

import sys

import rclpy

import vision_tools


def main():
    target = " ".join(sys.argv[1:]).strip()
    client = vision_tools.get_client()
    print(f"Asking the camera about: {target or 'the whole scene'} ...")
    answer = client.ask(target)
    if answer.get("error"):
        print(f"Error: {answer['error']}")
    else:
        if target:
            print(f"  target visible : {vision_tools.yes_no(answer['target_visible'])}")
            print(f"  position       : {answer['position']}")
            print(f"  size           : {answer['size']}")
        print(f"  path clear     : {vision_tools.yes_no(answer['path_clear'])}"
              f"   (obstacle ahead: {answer['obstacle_ahead']})")
        print(f"  description    : {answer['description']}")
        print(f"  model / time   : {answer['model']}, {answer['seconds']:.1f} s")
        print(f"  photo          : /workspace/{answer['image']}")
    vision_tools.close_client()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
