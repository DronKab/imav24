import rclpy
import time
from rclpy.node import Node

class State1(Node):
    def __init__(self):
        super().__init__("state1")
        self.get_logger().info("State 1 node started")

        time.sleep(10)
        rclpy.shutdown()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = State1()

        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()

        return "succeeded"
    except:
        return "aborted"

if __name__ == '__main__':
    main()