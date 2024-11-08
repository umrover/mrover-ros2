#!/usr/bin/env python3

"""
TODO(quintin): Document
"""

import sys

from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, protocol
from serial import Serial

import rclpy
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtcm_msgs.msg import Message as RTCMMessage


class BaseStationDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("basestation_gps_driver")

        self.declare_parameters(
            "",
            [
                ("port", Parameter.Type.STRING),
                ("baud", Parameter.Type.INTEGER),
            ],
        )
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        self.rtcm_pub = self.create_publisher(RTCMMessage, "rtcm", 10)

        self.svin_started = False
        self.svin_complete = False

        self.serial = Serial(port, baud, timeout=1)
        self.reader = UBXReader(self.serial, protfilter=UBX_PROTOCOL | RTCM3_PROTOCOL)

    def spin(self) -> None:
        while rclpy.ok():
            if self.serial.in_waiting:
                raw_msg, msg = self.reader.read()

                if not msg:
                    continue

                if protocol(raw_msg) == RTCM3_PROTOCOL:
                    self.rtcm_pub.publish(RTCMMessage(message=raw_msg))
                elif msg.identity == "NAV-SVIN":
                    if not self.svin_started and msg.active:
                        self.svin_started = True
                        self.get_logger().info("Base station survey-in started")
                    if not self.svin_complete and msg.valid:
                        self.svin_complete = True
                        self.get_logger().info(f"Base station survey-in complete, accuracy = {msg.meanAcc}")
                    if self.svin_started and not self.svin_complete:
                        self.get_logger().info(f"Current accuracy: {msg.meanAcc}")
                elif msg.identity == "NAV-PVT":
                    self.get_logger().info(
                        f"{'Valid' if msg.gnssFixOk else 'Invalid'} fix, {msg.numSV} satellites used"
                    )
                elif msg.identity == "NAV-SAT":
                    if (msg.gnssId == 0):
                        self.get_logger().info(f"GPS signal strength: {msg.cno}")
                    elif (msg.gnssId == 1):
                        self.get_logger().info(f"SBAS signal strength: {msg.cno}")
                    elif (msg.gnssId == 2):
                        self.get_logger().info(f"Galileo signal strength: {msg.cno}")
                    elif (msg.gnssId == 3):
                        self.get_logger().info(f"BeiDou signal strength: {msg.cno}")
                    elif (msg.gnssId == 5):
                        self.get_logger().info(f"QZSS signal strength: {msg.cno}")
                    elif (msg.gnssId == 6):
                        self.get_logger().info(f"GLONASS signal strength: {msg.cno}")
                        
            rclpy.spin_once(self, timeout_sec=0)

    def __del__(self) -> None:
        self.serial.close()


def main() -> None:
    try:
        rclpy.init(args=sys.argv)
        BaseStationDriverNode().spin()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
