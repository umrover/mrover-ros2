import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Point  # Modify this import if needed, based on your custom message type
from mrover.msg import (
    Throttle,
    IK,
    ControllerState,
    LED,
    StateMachineStateUpdate,
    GPSWaypoint,
    WaypointType,
    HeaterData,
    ScienceThermistors,
    Oxygen,
    Methane,
    UV,
)

class OxygenDataPublisher(Node):
    def __init__(self):
        super().__init__('oxygen_data_publisher')
        self.publisher = self.create_publisher(Oxygen, 'science_oxygen_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # Publishes every second

    def publish_data(self):
        msg = Oxygen()  # Modify to your message type if needed
        msg.percent = 50.0  # percent value
        msg.variance = 10.0  # variance value
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Oxygen => Percent: {msg.percent}, Variance => {msg.variance}')


class UVPublisher(Node):
    def __init__(self):
        super().__init__('uv_publisher')
        self.publisher = self.create_publisher(UV, 'science_uv_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # Publishes every second

    def publish_data(self):
        msg = UV() 
        msg.uv_index = 40.0 
        msg.variance = 10.0  # variance value
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing UV => Index: {msg.uv_index}, Variance: {msg.variance}')


# class TempPublisher(Node):
#     def __init__(self):
#         super().__init__('temp_data_publisher')
#         self.publisher = self.create_publisher(Temperature, 'science_temperature_data', 10)
#         self.timer = self.create_timer(1.0, self.publish_data)  # Publishes every second

#     def publish_data(self):
#         msg = Oxygen()  # Modify to your message type if needed
#         msg.temperature = 30.0  # percent value
#         msg.variance = 10.0  # variance value
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Publishing Temp => Temp: {msg.percent}, Variance: {msg.variance}')


# class HumidityPublisher(Node):
#     def __init__(self):
#         super().__init__('humidity_publisher')
#         self.publisher = self.create_publisher(RelativeHumidity, 'science_humidity_data', 10)
#         self.timer = self.create_timer(1.0, self.publish_data)  # Publishes every second

#     def publish_data(self):
#         msg = RelativeHumidity()  # Modify to your message type if needed
#         msg.relative_humidity = 5.0  # percent value
#         msg.variance = 10.0  # variance value
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Publishing Humidity => Percent: {msg.percent}, Variance: {msg.variance}')


def main(args=None):
    rclpy.init(args=args)
    node = OxygenDataPublisher()
    node2= UVPublisher()
    # node3 = TempPublisher()
    # node4 = HumidityPublisher()
    rclpy.spin(node)
    rclpy.spin(node2)
    # rclpy.spin(node3)
    # rclpy.spin(node4)
    rclpy.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()