#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
import argparse

from mrover.msg import Humidity, Temperature, Oxygen, UV, Ozone, CO2, Pressure, ControllerState, Throttle

SP_MOTORS = ["linear_actuator", "auger", "pump_0", "pump_1", "sensor_actuator"]


class MockSensorData(Node):
    def __init__(self, sensor_rate: float, controller_rate: float):
        super().__init__("mock_sensor_data")

        self.humidity_pub = self.create_publisher(Humidity, "/sp_humidity_data", 10)
        self.temp_pub = self.create_publisher(Temperature, "/sp_temp_data", 10)
        self.oxygen_pub = self.create_publisher(Oxygen, "/sp_oxygen_data", 10)
        self.uv_pub = self.create_publisher(UV, "/sp_uv_data", 10)
        self.ozone_pub = self.create_publisher(Ozone, "/sp_ozone_data", 10)
        self.co2_pub = self.create_publisher(CO2, "/sp_co2_data", 10)
        self.pressure_pub = self.create_publisher(Pressure, "/sp_pressure_data", 10)
        self.state_pub = self.create_publisher(ControllerState, "/sp_controller_state", 10)

        self.create_subscription(Throttle, "/sp_thr_cmd", self.on_throttle, 10)

        self.t = 0.0
        self.last_log_time = 0.0
        self.create_timer(1.0 / sensor_rate, self.publish_sensor_data)
        self.create_timer(1.0 / controller_rate, self.publish_controller_state)

        self.get_logger().info(f"Mock sensor data node started (sensors: {sensor_rate} Hz, controller: {controller_rate} Hz)")

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
        uv_msg.uv_index = max(0, min(11, int(5 + 5 * math.sin(self.t * 1.2) + random.uniform(-1, 1))))
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

    def publish_controller_state(self):
        msg = ControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "science"
        msg.names = SP_MOTORS
        msg.states = ["Armed"] * len(SP_MOTORS)
        msg.errors = [""] * len(SP_MOTORS)
        msg.positions = [0.0] * len(SP_MOTORS)
        msg.velocities = [0.0] * len(SP_MOTORS)
        msg.currents = [0.0] * len(SP_MOTORS)
        msg.limits_hit = [0] * len(SP_MOTORS)
        self.state_pub.publish(msg)

    def on_throttle(self, msg: Throttle):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            pairs = ", ".join(f"{n}={t:.2f}" for n, t in zip(msg.names, msg.throttles))
            self.get_logger().info(f"SP throttle: {pairs}")
            self.last_log_time = now


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock science sensor data and SP controller state")
    parser.add_argument("-r", "--rate", type=float, default=1.0, help="Sensor publishing rate in Hz (default: 1.0)")
    parser.add_argument("--controller-rate", type=float, default=10.0, help="Controller state rate in Hz (default: 10.0)")
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    try:
        node = MockSensorData(parsed_args.rate, parsed_args.controller_rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
