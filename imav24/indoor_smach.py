import rclpy
import smach
import smach_ros
from imav24 import state1
from imav24 import state2
from imav24 import aruco_control
from imav24 import line_follower
from imav24 import SetPhoto

from std_msgs.msg import Empty, Float32

from rclpy.node import Node

class IndoorSmach(Node):
    def __init__(self):
        super().__init__("indoor_smach")
        self.get_logger().info("State Machine node started")

        # Create publishers
        self.takeoff_pub = self.create_publisher(Empty, "/px4_driver/takeoff", 10)
        self.change_height_pub = self.create_publisher(Float32, "/px4_driver/target_height", 10)
        # Create state machine
        sq = smach.Sequence(outcomes=["succeeded", "aborted", "preempted"], connector_outcome="succeeded")

        # @mach.cb_interface(input_keys=['q'], output_keys=['xyz'], outcomes=['foo'])
                           
        # Add States
        with sq:
            smach.Sequence.add("INITIAL TAKEOFF", smach.CBState(self.takeoff, outcomes=["succeeded"]))
            smach.Sequence.add("STATE LINE_105", line_follower.NodeState(105))
            smach.Sequence.add("HEIGHT", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.5], outcomes=["succeeded"]))
            smach.Sequence.add("TAKE PHOTO", SetPhoto.NodeState())
            smach.Sequence.add("HEIGHT WINDOW", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1], outcomes=["succeeded"]))
            # estado aruco control yaw
            # smach.Sequence.add("STATE LINE_200", line_follower.NodeState(200))
            # smach.Sequence.add("STATE LINE_205", line_follower.NodeState(205))
            # smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(id, x, y, yaw, flag_land))
            smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(300))
            # smach.Sequence.add("STATE ARUCO_301", aruco_control.NodeState(301, 0.0, 0.0, 90))
            # smach.Sequence.add("STATE ARUCO_302", aruco_control.NodeState(302, 0.0, 0.0, 90))
            smach.Sequence.add("HEIGHT FOR LINE", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.5], outcomes=["succeeded"]))
            smach.Sequence.add("STATE LINE_400", line_follower.NodeState(400))
            smach.Sequence.add("STATE ARUCO_400", aruco_control.NodeState(400))
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

    def takeoff(self, userdata):
        self.get_logger().info("Publishing takeoff msg")
        self.takeoff_pub.publish(Empty())
        return "succeeded"
    
    def control_height(self, userdata, altura):
        msg = Float32()
        msg.data = altura
        self.get_logger().info(f"Changed height target to {msg.data}")
        self.change_height_pub.publish(msg)
        return "succeeded"
        

def main(args=None):

    rclpy.init(args=args)
    indoor_smach = IndoorSmach()
    rclpy.spin(indoor_smach)

    indoor_smach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()