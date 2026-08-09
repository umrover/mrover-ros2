#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mrover.srv import MorsePusher, Pusher
from morse_code import MorseCode

class MorsePusherNode(Node):
    ALPHANUMERIC_CHARACTERS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")

    def __init__(self):
        super().__init__("morse_pusher_service")

        self.callback_group = ReentrantCallbackGroup()

        # Make parameters to tune more easily
        self.declare_parameter("dot_duration", 0.2)
        self.declare_parameter("dash_duration", 0.6)
        self.declare_parameter("symbol_gap", 0.0)
        self.declare_parameter("letter_gap", 0.3)
        self.declare_parameter("pusher_timeout", 15.0)

        self.morse_code = MorseCode()
        self.pusher_client = self.create_client(Pusher,
            "pusher",
            callback_group=self.callback_group,)
        self.morse_service = self.create_service(MorsePusher,
            "morse_pusher",
            self.handle_morse_request,
            callback_group=self.callback_group,)
        self.busy = False

        self.get_logger().info("Initialized morse code pusher service")

    @classmethod
    def validate_input(cls, input_str: str) -> str:
        uppercase_text = input_str.strip().upper()

        for char in uppercase_text:
            if char not in cls.ALPHANUMERIC_CHARACTERS:
                raise ValueError(f"Invalid character {char}")

        return uppercase_text

    def get_timing_parameters(self) -> tuple[float, float, float, float, float]:
        dot_duration = self.get_parameter("dot_duration").value
        dash_duration = self.get_parameter("dash_duration").value
        symbol_gap = self.get_parameter("symbol_gap").value
        letter_gap = self.get_parameter("letter_gap").value
        pusher_timeout = self.get_parameter("pusher_timeout").value

        return (dot_duration, dash_duration, symbol_gap, letter_gap, pusher_timeout)

    def push_single_symbol(self, symbol: str, dot_duration: float, dash_duration: float, pusher_timeout: float) -> None:
        if symbol == ".":
            hold_duration = dot_duration
            symbol_name = "DOT"
        elif symbol == "-":
            hold_duration = dash_duration
            symbol_name = "DASH"
        else:
            raise ValueError(f"Invalid morse symbol {symbol}")

        request = Pusher.Request()
        request.start = True
        request.hold_duration = hold_duration

        self.get_logger().info(f"Pushing {symbol}")

        future = self.pusher_client.call_async(request)

        start_time = time.monotonic()

        while rclpy.ok() and not future.done():
            if (
                time.monotonic() - start_time
                >= pusher_timeout
            ):
                raise TimeoutError(
                    "/pusher service timed out"
                )

            time.sleep(0.01)

        response = future.result()

        if response is None or not response.finished:
            raise RuntimeError(
                "/pusher failed"
            )

    def execute_morse(self, morse:str) -> None:
        (dot_duration, dash_duration, symbol_gap, letter_gap, pusher_timeout) = self.get_timing_parameters()

        letters = morse.split()

        for letter_index, letter in enumerate(letters):
            for symbol_index, symbol in enumerate(letter):
                self.push_single_symbol(symbol, dot_duration, dash_duration, pusher_timeout)
                if symbol_index < len(letter) - 1:
                    time.sleep(symbol_gap)
            if letter_index < len(letters) - 1:
                time.sleep(letter_gap)

    def handle_morse_request(self, req: MorsePusher.Request, res: MorsePusher.Response) -> MorsePusher.Response:
        if self.busy:
            res.success = False
            res.morse = ""
            res.message = (
                "Morse pusher is already busy"
            )
            return res

        self.busy = True

        try:
            text = self.validate_input(
                req.input
            )

            morse = self.morse_code.encode(
                text
            )

            res.morse = morse

            self.get_logger().info(
                f"Input: {text}"
            )

            self.get_logger().info(
                f"Morse: {morse}"
            )

            if not self.pusher_client.wait_for_service(
                timeout_sec=2.0
            ):
                raise RuntimeError(
                    "/pusher service is not available"
                )

            self.execute_morse(
                morse
            )

            res.success = True
            res.message = (
                "Completed Morse code sequence"
            )

        except Exception as error:
            self.get_logger().error(
                str(error)
            )

            res.success = False
            res.morse = ""
            res.message = str(error)

        finally:
            self.busy = False

        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    executor.add_node(MorsePusherNode());
    executor.spin();
    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


