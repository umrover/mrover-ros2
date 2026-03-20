#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
import argparse

from mrover.msg import Humidity, Temperature, Oxygen, UV, Ozone, CO2, Pressure


class MockSensorDataPublisher(Node):
    def __init__(self, rate: float):
        super().__init__("mock_sensor_data_publisher")

        self.humidity_pub = self.create_publisher(Humidity, "/sp_humidity_data", 10)
        self.temp_pub = self.create_publisher(Temperature, "/sp_temp_data", 10)
        self.oxygen_pub = self.create_publisher(Oxygen, "/sp_oxygen_data", 10)
        self.uv_pub = self.create_publisher(UV, "/sp_uv_data", 10)
        self.ozone_pub = self.create_publisher(Ozone, "/sp_ozone_data", 10)
        self.co2_pub = self.create_publisher(CO2, "/sp_co2_data", 10)
        self.pressure_pub = self.create_publisher(Pressure, "/sp_pressure_data", 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_sensor_data)
        self.t = 0.0

        self.get_logger().info(f"Publishing mock sensor data at {rate} Hz")

    def publish_sensor_data(self):
        self.t += 0.3

        humidity_msg = Humidity()
        humidity_msg.relative_humidity = 50.0 + 30.0 * math.sin(self.t) + random.uniform(-5, 5)
        self.humidity_pub.publish(humidity_msg)

        temp_msg = Temperature()
        temp_msg.temperature = 25.0 + 15.0 * math.sin(self.t * 0.8) + random.uniform(-3, 3)
        self.temp_pub.publish(temp_msg)

        oxygen_msg = Oxygen()
        oxygen_msg.percent = 20.0 + 5.0 * math.sin(self.t * 0.6) + random.uniform(-1, 1)
        self.oxygen_pub.publish(oxygen_msg)

        uv_msg = UV()
        uv_msg.uv_index = int(5 + 5 * math.sin(self.t * 1.2) + random.uniform(-1, 1))
        uv_msg.uv_index = max(0, min(11, uv_msg.uv_index))
        self.uv_pub.publish(uv_msg)

        ozone_msg = Ozone()
        ozone_msg.ppb = 50.0 + 40.0 * math.sin(self.t * 0.7) + random.uniform(-5, 5)
        self.ozone_pub.publish(ozone_msg)

        co2_msg = CO2()
        co2_msg.ppm = 500.0 + 200.0 * math.sin(self.t * 0.5) + random.uniform(-20, 20)
        self.co2_pub.publish(co2_msg)

        pressure_msg = Pressure()
        pressure_msg.pressure = 101325.0 + 2000.0 * math.sin(self.t * 0.4) + random.uniform(-200, 200)
        self.pressure_pub.publish(pressure_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Publish mock sensor data for SP science sensors")
    parser.add_argument("-r", "--rate", type=float, default=1.0, help="Publishing rate in Hz (default: 1.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        publisher = MockSensorDataPublisher(parsed_args.rate)
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
