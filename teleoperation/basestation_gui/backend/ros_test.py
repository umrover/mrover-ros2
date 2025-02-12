import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class OxygenPublisher(Node):
    def __init__(self):
        super().__init__('oxygen_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'oxygen', 10)
        self.timer = self.create_timer(1.0, self.publish_oxygen_data)  # Publish every second
        
    def publish_oxygen_data(self):
        msg = Float32MultiArray()
        percent = random.uniform(18.0, 21.0)  # Simulating realistic oxygen levels
        variance = random.uniform(0.1, 0.5)   # Simulating some variance
        msg.data = [percent, variance]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: percent={percent:.2f}, variance={variance:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = OxygenPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
