#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import string
import argparse
import importlib
from rosidl_runtime_py.utilities import get_message


class RandomMessagePublisher(Node):
    def __init__(self, topic_name, msg_type, rate):
        super().__init__("random_message_publisher")

        self.msg_class = self._get_message_class(msg_type)

        self.publisher = self.create_publisher(self.msg_class, topic_name, 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_random_message)

        self.get_logger().info(f"Publishing random {msg_type} messages to {topic_name} at {rate} Hz")

    def _get_message_class(self, msg_type):
        try:
            return get_message(msg_type)
        except (ValueError, AttributeError, ModuleNotFoundError) as e:
            self.get_logger().error(
                f"Failed to import message type {msg_type}: {e}\n"
                f"Message type must be fully qualified (e.g., sensor_msgs/msg/NavSatFix, geometry_msgs/msg/Twist)"
            )
            raise

    def _generate_random_value(self, field_type):
        type_name = field_type.__name__ if hasattr(field_type, "__name__") else str(field_type)

        if type_name in ["bool", "boolean"]:
            return random.choice([True, False])
        elif type_name == "int8":
            return random.randint(-128, 127)
        elif type_name in ["uint8", "byte", "char"]:
            return random.randint(0, 255)
        elif type_name == "int16":
            return random.randint(-32768, 32767)
        elif type_name == "uint16":
            return random.randint(0, 65535)
        elif type_name == "int32":
            return random.randint(-2147483648, 2147483647)
        elif type_name == "uint32":
            return random.randint(0, 4294967295)
        elif type_name in ["int64", "int"]:
            return random.randint(-1000, 1000)
        elif type_name == "uint64":
            return random.randint(0, 1000)
        elif type_name in ["float", "float32", "float64", "double"]:
            return random.uniform(-100.0, 100.0)
        elif type_name in ["str", "string"]:
            return "".join(random.choices(string.ascii_letters + string.digits, k=10))
        else:
            return self._generate_random_message(field_type)

    def _generate_random_message(self, msg_class):
        msg = msg_class()

        if hasattr(msg, "get_fields_and_field_types"):
            fields = msg.get_fields_and_field_types()

            for field_name, field_type_str in fields.items():
                is_array = "sequence" in field_type_str or "[" in field_type_str

                if "[" in field_type_str and "]" in field_type_str:
                    array_size_str = field_type_str.split("[")[1].split("]")[0]
                    try:
                        fixed_array_size = int(array_size_str)
                    except ValueError:
                        fixed_array_size = None
                else:
                    fixed_array_size = None

                base_type_str = field_type_str.replace("sequence<", "").replace(">", "").split("[")[0].strip()

                try:
                    if "/" in base_type_str:
                        field_type = get_message(base_type_str)
                    else:
                        field_type = eval(base_type_str)
                except:
                    field_type = base_type_str

                if is_array:
                    if fixed_array_size is not None:
                        array_length = fixed_array_size
                    else:
                        array_length = random.randint(0, 5)
                    value = [self._generate_random_value(field_type) for _ in range(array_length)]
                else:
                    value = self._generate_random_value(field_type)

                setattr(msg, field_name, value)

        return msg

    def publish_random_message(self):
        msg = self._generate_random_message(self.msg_class)
        self.publisher.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Publish random ROS2 messages")
    parser.add_argument("topic", type=str, help="Topic name to publish to")
    parser.add_argument(
        "msg_type", type=str, help="Message type (e.g., std_msgs/msg/String or geometry_msgs/msg/Twist)"
    )
    parser.add_argument("-r", "--rate", type=float, default=1.0, help="Publishing rate in Hz (default: 1.0)")

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    try:
        publisher = RandomMessagePublisher(parsed_args.topic, parsed_args.msg_type, parsed_args.rate)
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
