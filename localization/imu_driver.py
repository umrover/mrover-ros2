import traceback
import sys

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy import Parameter
from rclpy.executors import ExternalShutdownException

import board
import busio

from adafruit_bno08x import {
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
}

from adafruit_bno08x.i2c import BNO08X_I2C
from geometry_msgs.msg import Quaternion, Vector3
from mrover.msg import calibrationStatus
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

BN0085_I2C_ADDRESS = 0x4A
IMU_RETRY_DURATION = 1

class ImuDriverNode(Node):

    def __init__(self) -> None:
        super().__init__("imu_driver")

        self.get_logger().info("IMU I2C driver starting...")

        self.declare_parameters(
            "",
            [
                ("frame_id", Parameter.Type.STRING),
                ("update_rate", Parameter.Type.INTEGER),
            ],
        )

        self.frame_id = self.get_parameter("frame_id").value
        self.update_rate = self.get_parameter("update_rate").value

        self.imu_calib_pub = self.create_publisher(Imu, "/imu/data", 1)
        self.imu_uncalib_pub = self.create_publisher(Imu, "/imu/data_raw", 1)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 1)
        self.calibration_status_pub = self.create_publisher(calibrationStatus, "/imu/calibration", 1)

        self.get_logger().info("initializing IMU I2C connection...")

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=BNO08X_I2C)


    def configure(self) -> None:

        try:
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

        except Exception as e:
            raise Exception() from e


    def parse_imu(self) -> None:
        
        header = Header(self.get_clock().now(), frame_id=self.frame_id)

        try:

            self.imu_calib_pub.publish(
                Imu(
                    header=header,
                    orientation=Quaternion(*self.bno.quaternion),
                    angular_velocity=Vector3(*self.bno.gyro),
                    linear_acceleration=Vector3(*self.bno.acceleration),
                ),
            )

            self.imu_uncalib_pub.publish(
                Imu(
                    header=header,
                    orientation=Quaternion(*self.bno.game_quaternion),
                ),
            )

            self.mag_pub.publish(
                MagneticField(
                    header=header,
                    magnetic_field=Vector3(*self.bno.magnetic),
                ),
            )

            c1, c2, c3 = self.bno.calibration_status

            self.calibration_status_pub.publish(
                calibrationStatus(
                    header=header,
                    magnetometer_calibration=0,
                    gyroscope_calibration=c2,
                    acceleration_calibration=c3,
                ),
            )

        except Exception as e:
            self.get_logger().warning(e)
            self.get_logger().warning(traceback.format_exc())
            start = self.get_clock().now()

            while(self.get_clock().now() - start < rclpy.duration.Duration(IMU_RETRY_DURATION)):
                try:
                    self.get_logger().info("Attempting to re-enable readings...", throttle_duration_sec=1.0)
                    self.bno._readings.clear()
                    self.configure()
                    self.get_logger().info("Restarted!")
                    break

                except Exception as e:
                    self.get_logger().warning("Re-enable failed...", throttle_duration_sec=2.0)

            else:
                self.get_logger().fatal("Failed to restart IMU driver, exiting...")



    def spin(self) -> None:
        self.get_logger().info("Starting IMU dynamic calibration...")

        # configure
        while(rclpy.ok()):

            self.bno.begin_calibration()

            try:
                self.get_logger().info("Configuring IMU reports...")
                self.configure()
                self.get_logger().info("IMU armed")
                break

            except Exception as e:
                self.get_logger().warning(f"Failed to enable all features: {e}, retrying...")
                self.get_logger().warning(traceback.format_exc())

        rate = self.create_rate(self.update_rate)

        # spin
        while (rclpy.ok()):
            self.parse_imu()
            rate.sleep()



def main() -> None:
    try:
        rclpy.init(args=sys.argv)
        ImuDriverNode().spin()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(status=1)

if __name__ == "__main__":
    main()

        









        