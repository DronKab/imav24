import rclpy
import smach
import smach_ros

from imav24 import state1
from imav24 import state2
from imav24 import aruco_control
from imav24 import line_follower

from rclpy.node import Node

class IndoorSmach(Node):
    def __init__(self):
        super().__init__("indoor_smach")
        self.get_logger().info("State Machine node started")

        # Create state machine
        sq = smach.Sequence(outcomes=["succeeded", "aborted", "preempted"], connector_outcome="succeeded")


        # Add States
        with sq:
            smach.Sequence.add("STATE LINE_105", line_follower.NodeState(aruco_finish=105))
            # cb sube
            # estado foto
            # cb altura ventana
            # smach.Sequence.add("STATE LINE_200", line_follower.NodeState(200))
            # smach.Sequence.add("STATE LINE_205", line_follower.NodeState(205))
            # smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(id, x, y, yaw))
            smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(300))
            # smach.Sequence.add("STATE ARUCO_301", aruco_control.NodeState(301, 0.0, 0.0, 90))
            # smach.Sequence.add("STATE ARUCO_302", aruco_control.NodeState(302, 0.0, 0.0, 90))
            # cd sube para ver linea
            smach.Sequence.add("STATE LINE_400", line_follower.NodeState(400))
            smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(400))
            # nodo toma el cono
            smach.Sequence.add("STATE ARUCO_405", aruco_control.NodeState(405))
            # nodo suelta cono
            smach.Sequence.add("STATE LINE_100", line_follower.NodeState(100))
            smach.Sequence.add("STATE2", state2.NodeState())
            smach.Sequence.add("STATE ARUCO_100", aruco_control.NodeState(100))
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