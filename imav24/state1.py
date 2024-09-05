import rclpy
import time
from rclpy.node import Node
from smach import State

class State1(Node):
    def __init__(self):
        super().__init__("state1")
        self.get_logger().info("State 1 node started")
        self.counter = 0

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        time.sleep(1)
        print(self.counter)
        self.counter += 1
        if self.counter == 10:
            raise ExitOk

# Code for making node runnable on ros
def main(args=None):
    rclpy.init(args=args)

    node = State1()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

# Code for making the node runnable on Smach
class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted"])
    def execute(self, userdata):
        try:

            node = State1()

            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except:
            return "aborted"