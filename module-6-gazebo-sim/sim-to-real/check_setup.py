"""Check that ROS 2 can see both the Gazebo robot and the real robot.

Run inside the container while sim_with_ground_truth.launch.py is running and
the Pico is on:
    python3 /workspace/sim-to-real/check_setup.py
    python3 /workspace/sim-to-real/check_setup.py --wheel-test   # also spin the real wheels
"""

import argparse

from robot_motion import RobotMotion

STATUS_NAMES = {0: "stopped", 1: "driving", 2: "stopped (command timeout)"}


def report(ok: bool, label: str, detail: str) -> bool:
    print(f"[{'OK' if ok else '--'}] {label}: {detail}")
    return ok


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel-test", action="store_true",
                        help="drive the real robot forward 1 s, then turn left 1 s")
    args = parser.parse_args()

    robots = RobotMotion()
    try:
        robots.wait_until_ready("sim", timeout=5.0)
        robots.wait_until_ready("real", timeout=5.0)

        pose = robots.sim_pose()
        sim_ok = report(pose is not None, "Gazebo robot (/odom)",
                        f"at ({pose[0]:.2f}, {pose[1]:.2f}), heading {pose[2]:.0f} deg"
                        if pose else "no data. Start the simulation.")
        robots.wait_for_ground_truth(timeout=5.0)
        truth = robots.sim_true_pose()
        sim_ok = report(truth is not None, "Gazebo true position (/sim/ground_truth)",
                        f"at ({truth[0]:.2f}, {truth[1]:.2f}), heading {truth[2]:.0f} deg"
                        if truth else "no data. Start the simulation with "
                        "sim_with_ground_truth.launch.py.") and sim_ok

        status = robots.real_drive_status()
        real_ok = report(robots.real_ready(), "Real robot (/pico/cmd_vel, /pico/drive/status)",
                         f"connected, {STATUS_NAMES.get(status, status)}" if status is not None
                         else "no data. Check the Pico's serial output and the micro-ROS Agent.")

        heading = robots.real_heading()
        report(heading is not None, "Real robot IMU (/pico/imu/heading)",
               f"{heading:.1f} deg" if heading is not None
               else "no data. Check the BNO055 wiring (the robot can still drive).")

        if not (sim_ok and real_ok):
            print("\nFix the items marked -- and run this check again.")
            return
        print("\nBoth robots are connected.")

        if args.wheel_test:
            answer = input("\nLift the real robot's wheels off the table. Type yes to spin them: ")
            if answer.strip().lower() != "yes":
                print("Wheel test skipped.")
                return
            # The robot is lifted, so its body cannot turn and the IMU heading does
            # not change. Judge this test by watching the wheels, not by numbers.
            tests = [
                ("Both wheels should spin FORWARD for 1 second...",
                 dict(linear=0.15, angular=0.0, seconds=1.0)),
                ("Now the RIGHT wheel should spin forward and the LEFT wheel backward "
                 "(a left turn) for 1 second...",
                 dict(linear=0.0, angular=1.0, seconds=1.0)),
            ]
            for description, command in tests:
                print(description)
                result = robots.move("real", **command)
                print("  Command sent and stopped." if result.completed
                      else f"  {result.message}")
            print("\nWatch the wheels, not the numbers: while the robot is lifted, its "
                  "heading does not change. If both tests looked right, the firmware works.")
    finally:
        robots.close()


if __name__ == "__main__":
    main()
