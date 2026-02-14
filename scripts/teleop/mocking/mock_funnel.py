#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import argparse
import math

from mrover.srv import ServoPosition

POSITION_TOLERANCE = 0.05


class MockFunnel(Node):
    def __init__(self, fail_rate: float):
        super().__init__("mock_funnel")

        self.current_position = 0.0
        self.fail_rate = fail_rate
        self.call_count = 0

        self.create_service(ServoPosition, "/sp_funnel_servo", self.on_servo_request)

        self.get_logger().info(
            f"Mock funnel servo started on /sp_funnel_servo (fail_rate={fail_rate:.0%})"
        )

    def on_servo_request(self, request, response):
        self.call_count += 1
        names = list(request.names)
        positions = list(request.positions)

        self.get_logger().info(
            f"Servo request: names={names}, "
            f"positions=[{', '.join(f'{p:.3f} ({math.degrees(p):.1f} deg)' for p in positions)}]"
        )

        at_tgts = []
        for name, target in zip(names, positions):
            should_fail = self.fail_rate > 0 and (self.call_count % max(1, int(1.0 / self.fail_rate)) == 0)

            if should_fail:
                self.get_logger().warn(f"  {name}: simulated failure (did not reach target)")
                at_tgts.append(False)
            else:
                self.current_position = target
                self.get_logger().info(
                    f"  {name}: moved to {target:.3f} rad ({math.degrees(target):.1f} deg)"
                )
                at_tgts.append(True)

        response.at_tgts = at_tgts
        return response


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock funnel servo service")
    parser.add_argument(
        "--fail-rate", type=float, default=0.0,
        help="Fraction of requests that simulate failure (0.0-1.0, default: 0.0)",
    )
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockFunnel(parsed_args.fail_rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
