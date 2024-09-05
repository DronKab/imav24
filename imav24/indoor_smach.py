import rclpy
import smach
import smach_ros

from imav24 import state1
from imav24 import state2

from rclpy.node import Node

class IndoorSmach(Node):
    def __init__(self):
        super().__init__("indoor_smach")
        self.get_logger().info("State Machine node started")

        # Create state machine
        sq = smach.Sequence(outcomes=["succeeded", "aborted", "preempted"], connector_outcome="succeeded")

        # Add States
        with sq:
            smach.Sequence.add("STATE1", state1.NodeState())
            smach.Sequence.add("STATE2", state2.NodeState())
            smach.Sequence.add("STATE1-2", state1.NodeState())

        # Start server for state machine visualization
        server = smach_ros.IntrospectionServer('indoor_smach_server', sq, '/SM_ROOT')
        server.start()

        # Execute state machine
        outcome = sq.execute()
        self.get_logger().info(f"State Machine ended with outcome {outcome}")

def main(args=None):
    rclpy.init(args=args)
    indoor_smach = IndoorSmach()
    rclpy.spin(indoor_smach)

    indoor_smach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()