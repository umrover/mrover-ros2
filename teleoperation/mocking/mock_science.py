#!/usr/bin/env python3

"""
Mock science platform controller state for SPDataTable.vue and sensor charts.

Exercises: sign changes, magnitude swings on positions/currents,
error/state cycling, limit toggling, stale periods.
Sensor data included for ScienceView charts.
"""

import math
import random
import argparse

import rclpy
from rclpy.node import Node

from mrover.msg import (
    ControllerState, Throttle,
    Humidity, Temperature, Oxygen, UV, Ozone, CO2, Pressure,
)

SP_MOTORS = ["linear_actuator", "auger", "pump_0", "pump_1", "sensor_actuator"]
STATES_CYCLE = ["Armed", "Armed", "Armed", "Disarmed"]
ERRORS_CYCLE = ["", "", "", "Stall"]


def oscillate(t: float, freq: float, amplitude: float, phase: float = 0.0) -> float:
    return amplitude * math.sin(t * freq + phase)


def square_wave(t: float, period: float, duty: float) -> bool:
    return (t % period) < duty


class MockScience(Node):
    def __init__(self, rate: float, sensor_rate: float, stale_period: float, stale_duration: float):
        super().__init__("mock_science")

        self.state_pub = self.create_publisher(ControllerState, "/sp_controller_state", 10)
        self.humidity_pub = self.create_publisher(Humidity, "/sp_humidity_data", 10)
        self.temp_pub = self.create_publisher(Temperature, "/sp_temp_data", 10)
        self.oxygen_pub = self.create_publisher(Oxygen, "/sp_oxygen_data", 10)
        self.uv_pub = self.create_publisher(UV, "/sp_uv_data", 10)
        self.ozone_pub = self.create_publisher(Ozone, "/sp_ozone_data", 10)
        self.co2_pub = self.create_publisher(CO2, "/sp_co2_data", 10)
        self.pressure_pub = self.create_publisher(Pressure, "/sp_pressure_data", 10)

        self.create_subscription(Throttle, "/sp_thr_cmd", self.on_throttle, 10)

        self.t = 0.0
        self.dt = 1.0 / rate
        self.stale_period = stale_period
        self.stale_duration = stale_duration
        self.last_log_time = 0.0

        self.create_timer(self.dt, self.tick_controller)
        self.create_timer(1.0 / sensor_rate, self.tick_sensors)
        self.get_logger().info(f"Mock science started (controller: {rate} Hz, sensors: {sensor_rate} Hz)")

    def in_stale(self) -> bool:
        if self.stale_period <= 0:
            return False
        return (self.t % self.stale_period) > (self.stale_period - self.stale_duration)

    def tick_controller(self):
        self.t += self.dt
        if self.in_stale():
            return

        stamp = self.get_clock().now().to_msg()
        n = len(SP_MOTORS)
        state_idx = int(self.t / 8.0) % len(STATES_CYCLE)
        error_idx = int(self.t / 12.0) % len(ERRORS_CYCLE)

        msg = ControllerState()
        msg.header.stamp = stamp
        msg.header.frame_id = "science"
        msg.names = list(SP_MOTORS)
        msg.states = [STATES_CYCLE[(state_idx + i) % len(STATES_CYCLE)] for i in range(n)]
        msg.errors = [ERRORS_CYCLE[(error_idx + i) % len(ERRORS_CYCLE)] for i in range(n)]
        msg.positions = [oscillate(self.t, 0.2 + i * 0.04, 60.0 + 50.0 * math.sin(self.t * 0.08 + i)) for i in range(n)]
        msg.velocities = [0.0] * n
        msg.currents = [oscillate(self.t, 0.35 + i * 0.05, 2.5, phase=i * 0.9) for i in range(n)]
        msg.limits_hit = [1 if square_wave(self.t + i * 4.0, 15.0, 2.0) else 0 for i in range(n)]
        self.state_pub.publish(msg)

    def tick_sensors(self):
        t = self.t

        msg = Humidity()
        msg.relative_humidity = 50.0 + 30.0 * math.sin(t) + random.uniform(-5, 5)
        self.humidity_pub.publish(msg)

        msg = Temperature()
        msg.temperature = 25.0 + 15.0 * math.sin(t * 0.8) + random.uniform(-3, 3)
        self.temp_pub.publish(msg)

        msg = Oxygen()
        msg.percent = 20.0 + 5.0 * math.sin(t * 0.6) + random.uniform(-1, 1)
        self.oxygen_pub.publish(msg)

        msg = UV()
        msg.uv_index = max(0, min(11, int(5 + 5 * math.sin(t * 1.2) + random.uniform(-1, 1))))
        self.uv_pub.publish(msg)

        msg = Ozone()
        msg.ppb = 50.0 + 40.0 * math.sin(t * 0.7) + random.uniform(-5, 5)
        self.ozone_pub.publish(msg)

        msg = CO2()
        msg.ppm = 500.0 + 200.0 * math.sin(t * 0.5) + random.uniform(-20, 20)
        self.co2_pub.publish(msg)

        msg = Pressure()
        msg.pressure = 101325.0 + 2000.0 * math.sin(t * 0.4) + random.uniform(-200, 200)
        self.pressure_pub.publish(msg)

    def on_throttle(self, msg: Throttle):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time > 1.0:
            pairs = ", ".join(f"{n}={t:.2f}" for n, t in zip(msg.names, msg.throttles))
            self.get_logger().info(f"SP throttle: {pairs}")
            self.last_log_time = now


def main(args=None):
    parser = argparse.ArgumentParser(description="Mock science controller state and sensors")
    parser.add_argument("-r", "--rate", type=float, default=10.0, help="Controller publish rate in Hz (default: 10)")
    parser.add_argument("--sensor-rate", type=float, default=1.0, help="Sensor publish rate in Hz (default: 1)")
    parser.add_argument("--stale-period", type=float, default=45.0, help="Stale cycle in seconds, 0 to disable (default: 45)")
    parser.add_argument("--stale-duration", type=float, default=3.0, help="Stale length in seconds (default: 3)")
    parsed = parser.parse_args()

    rclpy.init(args=args)
    try:
        rclpy.spin(MockScience(parsed.rate, parsed.sensor_rate, parsed.stale_period, parsed.stale_duration))
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
