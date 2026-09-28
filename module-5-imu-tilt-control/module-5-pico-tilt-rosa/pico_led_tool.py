import threading
import time

import rclpy
from langchain_core.tools import tool
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class PicoLedBridge:
    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()

        self._node = Node("rosa_pico_led_bridge")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._publisher = self._node.create_publisher(Bool, "/pico/led/set", qos)
        self._subscription = self._node.create_subscription(
            Bool,
            "/pico/led/state",
            self._on_state,
            qos,
        )
        self._call_lock = threading.Lock()
        self._state_received = False
        self._last_state = False

    def _on_state(self, message: Bool) -> None:
        self._last_state = message.data
        self._state_received = True

    def set_and_wait(self, state: bool, timeout_sec: float = 2.0) -> tuple[bool, str]:
        with self._call_lock:
            discovery_deadline = time.monotonic() + timeout_sec
            while self._publisher.get_subscription_count() == 0:
                remaining = discovery_deadline - time.monotonic()
                if remaining <= 0:
                    return False, "Pico LED is disconnected; no command subscriber was found."
                rclpy.spin_once(self._node, timeout_sec=min(0.1, remaining))

            self._state_received = False
            command = Bool()
            command.data = state
            self._publisher.publish(command)

            acknowledgment_deadline = time.monotonic() + timeout_sec
            while not self._state_received or self._last_state != state:
                remaining = acknowledgment_deadline - time.monotonic()
                if remaining <= 0:
                    return False, "Pico LED command timed out without matching acknowledgment."
                rclpy.spin_once(self._node, timeout_sec=min(0.1, remaining))

            return True, f"Pico LED acknowledged {'on' if state else 'off'}."


_bridge: PicoLedBridge | None = None
_bridge_lock = threading.Lock()


def _get_bridge() -> PicoLedBridge:
    global _bridge
    with _bridge_lock:
        if _bridge is None:
            _bridge = PicoLedBridge()
        return _bridge


@tool
def set_pico_led(state: bool) -> str:
    """Turn the Pico 2 W onboard LED on or off and verify its reported state."""
    _, result = _get_bridge().set_and_wait(state, timeout_sec=2.0)
    return result