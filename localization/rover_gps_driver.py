#!/usr/bin/env python3

"""
Reads and parses NMEA messages from the onboard GPS to provide
location data to the rover over LCM (/gps). Subscribes to
/rtcm and passes RTCM messages to the onboard gps to
acquire an RTK fix.
"""

from __future__ import annotations

import sys

import serial
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL

import rclpy
from mrover.msg import FixStatus, SatelliteSignal, FixType
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtcm_msgs.msg import Message as RTCMMessage
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header


class RoverGpsDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("gps_driver")

        self.declare_parameters(
            "",
            [
                ("port_ublox", Parameter.Type.STRING),
                ("baud_ublox", Parameter.Type.INTEGER),
            ],
        )
        self.port = self.get_parameter("port_ublox").value
        self.baud = self.get_parameter("baud_ublox").value

        self.gps_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.gps_status_pub = self.create_publisher(FixStatus, "gps_fix_status", 10)
        self.satellite_signal_pub = self.create_publisher(SatelliteSignal, "gps/satellite_signal", 10)

        self.base_station_sub = self.create_subscription(RTCMMessage, "rtcm", self.process_rtcm, 10)

        self.serial = serial.Serial(self.port, self.baud)
        self.reader = UBXReader(self.serial, protfilter=UBX_PROTOCOL | RTCM3_PROTOCOL)

    def process_rtcm(self, data: RTCMMessage) -> None:
        self.serial.write(data.message)

    def parse_ubx_message(self, msg) -> None:
        if not msg:
            return

        match msg.identity:
            case "RXM-RTCM":
                match msg.msgUsed:
                    case 0:
                        self.get_logger().warn("RTCM usage unknown")
                    case 1:
                        self.get_logger().warn("RTCM message not used")
                    case 2:
                        self.get_logger().debug("RTCM message successfully used by receiver")

            case "NAV-PVT":
                if msg.lat == 0 or msg.lon == 0:
                    self.get_logger().warn("Zero satellite fix. Are we inside?")
                    return

                self.gps_pub.publish(
                    NavSatFix(
                        header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
                        latitude=msg.lat,
                        longitude=msg.lon,
                        altitude=float(msg.hMSL),
                    )
                )

                self.gps_status_pub.publish(
                    FixStatus(
                        header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
                        fix_type=FixType(fix=msg.carrSoln),
                    )
                )

                if msg.diffSoln == 1:
                    self.get_logger().debug("Differential correction applied")
                if msg.carrSoln == 0:
                    self.get_logger().warn("No RTK")
                elif msg.carrSoln == 1:
                    self.get_logger().debug("Floating RTK Fix")
                elif msg.carrSoln == 2:
                    self.get_logger().debug("RTK FIX")

            case "NAV-SAT":
                for i in range(msg.numSvs):
                    gnssId = getattr(msg, f"gnssId_{i+1:02d}")
                    cno = getattr(msg, f"cno_{i+1:02d}")
                    if gnssId == 0:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="GPS", signal_strength=cno))
                    elif gnssId == 1:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="SBAS", signal_strength=cno))
                    elif gnssId == 2:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="Galileo", signal_strength=cno))
                    elif gnssId == 3:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="BeiDou", signal_strength=cno))
                    elif gnssId == 5:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="QZSS", signal_strength=cno))
                    elif gnssId == 6:
                        self.satellite_signal_pub.publish(SatelliteSignal(constellation="GLONASS", signal_strength=cno))

            case "NAV-STATUS":
                pass

    def spin(self) -> None:
        while rclpy.ok():
            if self.serial.in_waiting:
                raw, msg = self.reader.read()
                self.parse_ubx_message(msg)
            rclpy.spin_once(self, timeout_sec=0)

    def __del__(self) -> None:
        self.serial.close()


def main() -> None:
    try:
        rclpy.init(args=sys.argv)
        RoverGpsDriverNode().spin()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
