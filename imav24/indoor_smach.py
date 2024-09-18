import rclpy
import smach
import smach_ros
import time 
from imav24 import state1
from imav24 import state2
from imav24 import start_msg_node
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
                           
        with sq:
            smach.Sequence.add("WAIT_FOR_START_MSG", start_msg_node.NodeState())
            smach.Sequence.add("INITIAL TAKEOFF", smach.CBState(self.takeoff, outcomes=["succeeded"]))
            smach.Sequence.add("HEIGHT_takeoff", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[0.5], outcomes=["succeeded"]))
            smach.Sequence.add("DELAY_height_ctrl_1", smach.CBState(self.delay, input_keys=["secs"], cb_args=[3], outcomes=["succeeded"]))
            smach.Sequence.add("STATE LINE_105", line_follower.NodeState(105, False))
            smach.Sequence.add("STATE ARUCO_105", aruco_control.NodeState(105, 0.0, 0.0, 270, 0))
            smach.Sequence.add("HEIGHT_photo", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.1], outcomes=["succeeded"]))
            smach.Sequence.add("TAKE PHOTO", SetPhoto.NodeState())
            smach.Sequence.add("HEIGHT_window", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.0], outcomes=["succeeded"]))
            smach.Sequence.add("DELAY_height_ctrl_2", smach.CBState(self.delay, input_keys=["secs"], cb_args=[5], outcomes=["succeeded"]))
            smach.Sequence.add("STATE ARUCO_105_turn", aruco_control.NodeState(105, 0.0, 0.0, 90, 0))
            smach.Sequence.add("STATE LINE_205", line_follower.NodeState(205, False))
            smach.Sequence.add("STATE ARUCO_205", aruco_control.NodeState(205, 0.3, 0.0, 90, 0))
            smach.Sequence.add("HEIGHT_platform", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.8], outcomes=["succeeded"]))
            smach.Sequence.add("STATE LINE_300", line_follower.NodeState(300, False))
            smach.Sequence.add("STATE ARUCO_300", aruco_control.NodeState(300, 0.0, 0.0, 180, 1))
            # smach.Sequence.add("STATE LINE_301", line_follower.NodeState(301, False))
            # smach.Sequence.add("STATE ARUCO_301", aruco_control.NodeState(301, 0.0, 0.0, 90, 1))
            smach.Sequence.add("WAIT_FOR_START_MSG_PL", start_msg_node.NodeState())
            smach.Sequence.add("PLATFORM_takeoff_1", smach.CBState(self.takeoff, outcomes=["succeeded"]))
            smach.Sequence.add("DELAY_takeoff_1", smach.CBState(self.delay, input_keys=["secs"], cb_args=[5], outcomes=["succeeded"]))
            smach.Sequence.add("STATE ARUCO_301", aruco_control.NodeState(301, 0.0, 0.0, 90, 0))
            smach.Sequence.add("STATE ARUCO_302", aruco_control.NodeState(302, 0.0, 0.0, 180, 0))
            smach.Sequence.add("STATE LINE_400", line_follower.NodeState(400, False))
            smach.Sequence.add("HEIGHT_2_WINDOW", smach.CBState(self.control_height, input_keys=["altura"], cb_args=[1.0], outcomes=["succeeded"]))
            smach.Sequence.add("STATE LINE_BLUE_RED", line_follower.NodeState(300, True))
            smach.Sequence.add("STATE LINE_RED_BLUE", line_follower.NodeState(100, False))
            smach.Sequence.add("STATE ARUCO_100", aruco_control.NodeState(100, 0.0, 0.0, 180, 1))

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
    
    def delay(self, userdata, secs):
        seconds = secs
        time.sleep(seconds)
        return "succeeded"
    
        

def main(args=None):

    rclpy.init(args=args)
    indoor_smach = IndoorSmach()
    rclpy.spin(indoor_smach)

    indoor_smach.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()