import rclpy
import time
from rclpy.node import Node
from smach import State
from rclpy.executors import SingleThreadedExecutor

class State2(Node):
    def __init__(self):
        super().__init__("state2")
        self.get_logger().info("State 2 node started")
        time.sleep(5)
        raise ExitOk

# Code for standard run on ros2
def main(args=None):
    rclpy.init(args=args)

    node = State2()

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

            node = State2()

            rclpy.spin( node )
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except:
            node
        
        node.destroy_node()
        return "aborted"