"""Send new PID gains to the Pico (no reflashing needed).

Run inside the container:
    python3 /workspace/pid/set_gains.py 1.0 5.0 0.0              # Kp Ki Kd
    python3 /workspace/pid/set_gains.py 1.0 5.0 0.0 --no-ff      # feed-forward off
    python3 /workspace/pid/set_gains.py 1.0 5.0 0.0 --no-ramp    # pure steps (for tuning)

Gains reset to the firmware defaults when the Pico restarts.
"""

import argparse
import time

import rclpy
from std_msgs.msg import Float32MultiArray


def send_gains(node, kp, ki, kd, feed_forward=True, ramp=True, wait=3.0):
    pub = node.create_publisher(Float32MultiArray, "/pico/pid/gains", 10)
    end = time.time() + wait
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        return False
    msg = Float32MultiArray()
    msg.data = [float(kp), float(ki), float(kd), 1.0 if feed_forward else 0.0,
                1.0 if ramp else 0.0]
    for _ in range(3):        # send a few times in case one is lost over Wi-Fi
        pub.publish(msg)
        time.sleep(0.05)
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("kp", type=float)
    parser.add_argument("ki", type=float)
    parser.add_argument("kd", type=float)
    parser.add_argument("--no-ff", action="store_true", help="turn feed-forward off")
    parser.add_argument("--no-ramp", action="store_true", help="turn the setpoint ramp off")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("set_gains")
    ok = send_gains(node, args.kp, args.ki, args.kd, not args.no_ff, not args.no_ramp)
    print(f"Sent Kp={args.kp} Ki={args.ki} Kd={args.kd} feed-forward={'off' if args.no_ff else 'on'} "
          f"ramp={'off' if args.no_ramp else 'on'}" if ok else
          "Nobody is listening on /pico/pid/gains. Is the Pico running the PID firmware?")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
