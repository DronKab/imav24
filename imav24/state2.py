import rclpy
import time
from rclpy.node import Node

class State1(Node):
    def __init__(self):
        super().__init__("state2")
        self.get_logger().info("State 2 node started")
        time.sleep(20)

def main(args=None):
    rclpy.init(args=args)
    node = State1()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()