"""
Integration test for arm controls: WebSocket -> Backend -> ROS topic

This test verifies the full data flow:
1. WebSocket message sent to /arm endpoint
2. Backend processes the ra_controller message
3. ROS topic /arm_thr_cmd receives correct Throttle message

Prerequisites:
- Backend server running (localhost:8000)
- ROS 2 environment sourced
"""

import asyncio
import threading
import time
from typing import Optional
from dataclasses import dataclass, field

import pytest
import websocket
import msgpack
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import requests

from mrover.msg import Throttle


@dataclass
class ReceivedThrottle:
    names: list[str] = field(default_factory=list)
    throttles: list[float] = field(default_factory=list)
    timestamp: float = 0.0


class ThrottleSubscriber(Node):
    """ROS 2 node that subscribes to /arm_thr_cmd and stores received messages."""

    def __init__(self):
        super().__init__('test_arm_throttle_subscriber')
        self.received_messages: list[ReceivedThrottle] = []
        self.subscription = self.create_subscription(
            Throttle,
            '/arm_thr_cmd',
            self._callback,
            10
        )

    def _callback(self, msg: Throttle):
        self.received_messages.append(ReceivedThrottle(
            names=list(msg.names),
            throttles=list(msg.throttles),
            timestamp=time.time()
        ))

    def clear_messages(self):
        self.received_messages.clear()

    def wait_for_message(self, timeout: float = 5.0) -> Optional[ReceivedThrottle]:
        start = time.time()
        initial_count = len(self.received_messages)
        while time.time() - start < timeout:
            if len(self.received_messages) > initial_count:
                return self.received_messages[-1]
            time.sleep(0.05)
        return None


@pytest.fixture(scope='module')
def ros_context():
    """Initialize ROS 2 context for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def throttle_subscriber(ros_context):
    """Create a throttle subscriber node with its own executor thread."""
    node = ThrottleSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    yield node

    executor.shutdown()
    node.destroy_node()


def set_arm_mode(mode: str) -> bool:
    """Set the arm mode via REST API."""
    try:
        response = requests.post(
            'http://localhost:8000/api/arm/ra_mode',
            json={'mode': mode},
            timeout=5.0
        )
        return response.status_code == 200
    except requests.RequestException:
        return False


def send_arm_controller_input(axes: list[float], buttons: list[float]) -> bool:
    """Send controller input via WebSocket to the arm endpoint."""
    try:
        ws = websocket.create_connection(
            'ws://localhost:8000/arm',
            timeout=5.0
        )
        message = msgpack.packb({
            'type': 'ra_controller',
            'axes': axes,
            'buttons': buttons
        })
        ws.send(message, opcode=websocket.ABNF.OPCODE_BINARY)
        ws.close()
        return True
    except Exception as e:
        print(f'WebSocket error: {e}')
        return False


class TestArmRosIntegration:
    """Test suite for arm controls ROS integration."""

    @pytest.fixture(autouse=True)
    def setup(self, throttle_subscriber):
        """Clear messages before each test."""
        throttle_subscriber.clear_messages()
        time.sleep(0.1)

    def test_throttle_mode_publishes_joint_commands(self, throttle_subscriber):
        """
        Test that controller input in throttle mode publishes to /arm_thr_cmd.

        Expected behavior:
        - LEFT_X (axes[0]) controls joint_a
        - LEFT_Y (axes[1]) controls joint_b
        - RIGHT_Y (axes[3]) controls joint_c
        """
        assert set_arm_mode('throttle'), 'Failed to set arm mode to throttle'
        time.sleep(0.2)

        test_axes = [0.5, -0.7, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17

        assert send_arm_controller_input(test_axes, test_buttons), 'Failed to send controller input'

        msg = throttle_subscriber.wait_for_message(timeout=3.0)
        assert msg is not None, 'No message received on /arm_thr_cmd topic'

        assert 'joint_a' in msg.names, 'joint_a not in throttle message'
        assert 'joint_b' in msg.names, 'joint_b not in throttle message'
        assert 'joint_c' in msg.names, 'joint_c not in throttle message'

        joint_a_idx = msg.names.index('joint_a')
        joint_b_idx = msg.names.index('joint_b')
        joint_c_idx = msg.names.index('joint_c')

        # joint_a: LEFT_X (0.5) with scale -1.0 and quadratic filter
        # After deadzone (0.18) and quadratic: should be negative
        assert msg.throttles[joint_a_idx] < 0, 'joint_a should be negative (scale -1.0)'

        # joint_b: LEFT_Y (-0.7) with scale 0.8 and quadratic filter
        # After processing: should be negative (negative input * positive scale)
        assert msg.throttles[joint_b_idx] < 0, 'joint_b should be negative'

        # joint_c: RIGHT_Y (0.8) with scale 1.0 and quadratic filter
        # After processing: should be positive
        assert msg.throttles[joint_c_idx] > 0, 'joint_c should be positive'

        print(f'Received throttle: names={msg.names}, throttles={msg.throttles}')

    def test_disabled_mode_no_publish(self, throttle_subscriber):
        """Test that disabled mode does not publish throttle commands."""
        assert set_arm_mode('disabled'), 'Failed to set arm mode to disabled'
        time.sleep(0.2)

        test_axes = [0.5, -0.7, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17

        assert send_arm_controller_input(test_axes, test_buttons), 'Failed to send controller input'

        msg = throttle_subscriber.wait_for_message(timeout=1.0)
        assert msg is None, 'Should not receive message in disabled mode'

    def test_zero_input_within_deadzone(self, throttle_subscriber):
        """Test that small inputs within deadzone produce zero throttle."""
        assert set_arm_mode('throttle'), 'Failed to set arm mode to throttle'
        time.sleep(0.2)

        # All axes within deadzone (0.18)
        test_axes = [0.1, -0.1, 0.05, 0.15, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17

        assert send_arm_controller_input(test_axes, test_buttons), 'Failed to send controller input'

        msg = throttle_subscriber.wait_for_message(timeout=3.0)
        assert msg is not None, 'No message received on /arm_thr_cmd topic'

        joint_a_idx = msg.names.index('joint_a')
        joint_b_idx = msg.names.index('joint_b')
        joint_c_idx = msg.names.index('joint_c')

        # All stick inputs should be filtered to 0 by deadzone
        assert abs(msg.throttles[joint_a_idx]) < 0.01, 'joint_a should be ~0 (within deadzone)'
        assert abs(msg.throttles[joint_b_idx]) < 0.01, 'joint_b should be ~0 (within deadzone)'
        assert abs(msg.throttles[joint_c_idx]) < 0.01, 'joint_c should be ~0 (within deadzone)'


    def test_joint_a_alignment_left_x(self, throttle_subscriber):
        """Verify LEFT_X (axes[0]) controls joint_a with scale -1.0."""
        assert set_arm_mode('throttle')
        time.sleep(0.2)

        test_axes = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17
        assert send_arm_controller_input(test_axes, test_buttons)

        msg = throttle_subscriber.wait_for_message(timeout=3.0)
        assert msg is not None

        joint_a_idx = msg.names.index('joint_a')
        joint_b_idx = msg.names.index('joint_b')
        joint_c_idx = msg.names.index('joint_c')

        assert msg.throttles[joint_a_idx] < -0.1, 'joint_a should be negative (positive input * -1.0 scale)'
        assert abs(msg.throttles[joint_b_idx]) < 0.01, 'joint_b should be ~0'
        assert abs(msg.throttles[joint_c_idx]) < 0.01, 'joint_c should be ~0'

    def test_joint_b_alignment_left_y(self, throttle_subscriber):
        """Verify LEFT_Y (axes[1]) controls joint_b with scale 0.8."""
        assert set_arm_mode('throttle')
        time.sleep(0.2)

        test_axes = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17
        assert send_arm_controller_input(test_axes, test_buttons)

        msg = throttle_subscriber.wait_for_message(timeout=3.0)
        assert msg is not None

        joint_a_idx = msg.names.index('joint_a')
        joint_b_idx = msg.names.index('joint_b')
        joint_c_idx = msg.names.index('joint_c')

        assert abs(msg.throttles[joint_a_idx]) < 0.01, 'joint_a should be ~0'
        assert msg.throttles[joint_b_idx] > 0.1, 'joint_b should be positive (positive input * 0.8 scale)'
        assert abs(msg.throttles[joint_c_idx]) < 0.01, 'joint_c should be ~0'

    def test_joint_c_alignment_right_y(self, throttle_subscriber):
        """Verify RIGHT_Y (axes[3]) controls joint_c with scale 1.0."""
        assert set_arm_mode('throttle')
        time.sleep(0.2)

        test_axes = [0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0]
        test_buttons = [0.0] * 17
        assert send_arm_controller_input(test_axes, test_buttons)

        msg = throttle_subscriber.wait_for_message(timeout=3.0)
        assert msg is not None

        joint_a_idx = msg.names.index('joint_a')
        joint_b_idx = msg.names.index('joint_b')
        joint_c_idx = msg.names.index('joint_c')

        assert abs(msg.throttles[joint_a_idx]) < 0.01, 'joint_a should be ~0'
        assert abs(msg.throttles[joint_b_idx]) < 0.01, 'joint_b should be ~0'
        assert msg.throttles[joint_c_idx] > 0.1, 'joint_c should be positive (positive input * 1.0 scale)'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
