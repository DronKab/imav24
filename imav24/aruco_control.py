import rclpy
import numpy
import math
from smach import State
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
from aruco_opencv_msgs.msg import ArucoDetection

class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted"], input_keys=['aruco_go'])
    def execute(self, userdata):
        try:
            node = ArucoControl(userdata.aruco_go)
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except:
            return "aborted"

class ArucoControl(Node):
    def __init__(self, userdata):
        super().__init__("aruco_control")
        self.get_logger().info("Started Aruco Control ...")


        self.max_vel = 4.0
        self.max_vel_z = -0.1
        self.max_vel_yaw = 0.2
        self.aruco_id = 0

        self.last_known_x = None
        self.last_known_y = None
        self.last_known_z = None
        # self.last_known_yaw = None
        self.aruco_visible = False

        # Received parameters
        # self.aruco_goal = 100 # start
        # self.aruco_goal = 300 # first platform
        # self.aruco_goal = 301 # second platform
        # self.aruco_goal = 302 # third platform
        # self.aruco_goal = 400 # cone collection
        # self.aruco_goal = 405 # cone placement 
        self.aruco_goal = userdata.aruco_go

        self.roll = 0 # en grados 
        self.roll = math.radians(self.roll)
        self.pitch = 0 # en grados 
        self.pitch = math.radians(self.pitch)
        self.yaw = -90 # en grados 
        self.yaw = math.radians(self.yaw)

        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)

        self.vel_pub = self.create_publisher(Twist, "/px4_driver/cmd_vel", 10)
        self.land_pub = self.create_publisher(Empty, "/px4_driver/land", 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        self.q2 = self.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.get_logger().info(f"q2 : {self.q2}")

        if (self.aruco_goal == 300 or self.aruco_goal == 301 or self.aruco_goal == 302 or self.aruco_goal == 100):
            self.declare_parameter("do_height_control", False)
            self.x_distance = 0.0
            self.y_distance = 0.0

            # X variables
            self.px_gain = 0.9
            self.dx_gain = 0.1
            self.nx_filter = 0.1

            # Y variables
            self.py_gain = 0.9
            self.dy_gain = 0.1
            self.ny_filter = 0.1

            # Z variables
            self.pz_gain = 0.6
            self.dz_gain = 0.1
            self.nz_filter = 0.1

            # Yaw variables
            self.pyaw_gain = 0.9
            self.dyaw_gain = 0.1
            self.nyaw_filter = 0.1

        else:
            if self.aruco_goal == 400:
                self.declare_parameter("do_height_control", False)
                self.x_distance = -0.4
                self.y_distance = 0.0

            else:
                self.declare_parameter("do_height_control", True)
                self.x_distance = 0.0
                self.y_distance = 0.0

            # X variables
            self.px_gain = 0.5
            self.dx_gain = 0.06
            self.nx_filter = 0.1

            # Y variables 
            self.py_gain = 0.5
            self.dy_gain = 0.06
            self.ny_filter = 0.1

            # Z variables #
            self.pz_gain = 0.3
            self.dz_gain = 0.03
            self.nz_filter = 0.1

            # Yaw variables
            self.pyaw_gain = 0.5
            self.dyaw_gain = 0.06
            self.nyaw_filter = 0.1
        
        self.get_logger().info(f"PX Gain : {self.px_gain}")
        self.get_logger().info(f"DX Gain : {self.dx_gain}")
        self.get_logger().info(f"NX Coefficient : {self.nx_filter}")
        self.get_logger().info(f"PY Gain : {self.py_gain}")
        self.get_logger().info(f"DY Gain : {self.dy_gain}")
        self.get_logger().info(f"NY Coefficient : {self.ny_filter}")
        self.get_logger().info(f"PZ Gain : {self.pz_gain}")
        self.get_logger().info(f"DZ Gain : {self.dz_gain}")
        self.get_logger().info(f"NZ Coefficient : {self.nz_filter}")
        self.get_logger().info(f"PYaw Gain : {self.pyaw_gain}")
        self.get_logger().info(f"DYaw Gain : {self.dyaw_gain}")
        self.get_logger().info(f"NYaw Coefficient : {self.nyaw_filter}")
        
        self.q1 = Quaternion()
        self.x_error = 0.10
        self.x_error_1 = 0.0
        self.x_output = 0.0
        self.x_output_1 = 0.0
        self.y_error = 0.0
        self.y_error_1 = 0.0
        self.y_output = 0.0
        self.y_output_1 = 0.0
        self.z_error = 0.0
        self.z_error_1 = 0.0
        self.z_output = 0.0
        self.z_output_1 = 0.0
        self.yaw_error = 0.0
        self.yaw_error_1 = 0.0
        self.yaw_output = 0.0
        self.yaw_output_1 = 0.0

        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value
        self.ts = 0.064

        self.heartbeat_timer = self.create_timer(self.ts, self.control)

    
    def aruco_callback(self, msg):
        aruco_index = 0
        if len(msg.markers) > 0: 
            for i in range(0, len(msg.markers)):
                if msg.markers[i].marker_id == self.aruco_goal:
                    self.aruco_id = msg.markers[i].marker_id
                    aruco_index = i
                    break

            if msg.markers[aruco_index].marker_id == self.aruco_goal:
                self.x_error = -msg.markers[aruco_index].pose.position.y + self.x_distance
                self.y_error = -msg.markers[aruco_index].pose.position.x + self.y_distance
                self.z_error = -msg.markers[aruco_index].pose.position.z
                self.aruco_id = msg.markers[aruco_index].marker_id
                self.q1 = msg.markers[aruco_index].pose.orientation

                self.last_known_x = self.x_error
                self.last_known_y = self.y_error
                self.last_known_z = self.z_error
                self.aruco_visible = True 

            else:
                self.aruco_visible = False

        else:
            self.aruco_visible = False


    def control(self):
        msg = Twist()
        self.get_logger().info(f"id Aruco = {self.aruco_id}")

        # calculos para control de yaw
        q2_conj = self.conj_quat(self.q2)
        q_multip = self.hammilton(self.q1, q2_conj)
        _, _, self.yaw_error = self.euler_from_quaternion(q_multip)
        yaw_error_deg = math.degrees(self.yaw_error)

        if not self.aruco_visible and self.last_known_x is not None:
            self.x_error = self.last_known_x * 0.1
            self.y_error = self.last_known_y * 0.1
            self.z_error = self.last_known_z * 0.01
            self.yaw_error = 0.0

        self.get_logger().info("Entering control")
        self.get_logger().info(f"Erores: x={self.x_error}, y={self.y_error},  z={self.z_error}, yaw={yaw_error_deg}")

        # Control PD en X y Y
        if abs(yaw_error_deg) < 5:
            px_action = self.x_error * self.px_gain
            # ix_action = self.x_output_1 + self.x_error * self.ix_gain * self.ts
            dx_action = self.x_output_1 * (self.nx_filter * self.dx_gain * (self.x_error - self.x_error_1)) / (1 + self.nx_filter * self.ts)
            # self.x_output = float(px_action + ix_action + dx_action)
            self.x_output = float(px_action + dx_action)
            self.x_error_1 = self.x_error * 1.0
            self.x_output_1 = self.x_output * 1.0
            
            py_action = self.y_error * self.py_gain
            # iy_action = self.y_output_1 + self.y_error * self.iy_gain * self.ts
            dy_action = self.y_output_1 * (self.ny_filter * self.dy_gain * (self.y_error - self.y_error_1)) / (1 + self.ny_filter * self.ts)
            # self.y_output = float(py_action + iy_action + dy_action)
            self.y_output = float(py_action + dy_action)
            self.y_error_1 = self.y_error * 1.0
            self.y_output_1 = self.y_output * 1.0

        else:
            self.do_height_control = True
            self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value
            self.x_error = 0.0
            self.x_output = 0.0
            self.y_error = 0.0
            self.y_output = 0.0
            self.z_error = 0.0
            self.z_output = 0.0

        
        # Control PD en z
        if abs(self.x_error) < 0.1 and abs(self.y_error) < 0.1 and abs(yaw_error_deg) < 5:
            # Control PD en Z
            pz_action = self.z_error * self.pz_gain
            # iz_action = self.z_output_1 + self.z_error * self.iz_gain * self.ts
            dz_action = self.z_output_1 * (self.nz_filter * self.dz_gain * (self.z_error - self.z_error_1)) / (1 + self.nz_filter * self.ts)
            self.z_output = float(pz_action + dz_action)
            # self.z_output = float(pz_action + iz_action + dz_action)
        else:
            self.do_height_control = True
            self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value
            self.z_error = 0.0
            self.z_output = 0.0
        
        self.z_error_1 = self.z_error * 1.0
        self.z_output_1 = self.z_output * 1.0
        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        # Control PD en Yaw
        pyaw_action = self.yaw_error * self.pyaw_gain
        # iyaw_action = self.yaw_output_1 + self.yaw_error * self.iyaw_gain * self.ts
        dyaw_action = self.yaw_output_1 * (self.nyaw_filter * self.dyaw_gain * (self.yaw_error - self.yaw_error_1)) / (1 + self.nyaw_filter * self.ts)
        # self.yaw_output = float(pyaw_action + iyaw_action + dyaw_action)
        self.yaw_output = float(pyaw_action + dyaw_action)
        self.yaw_error_1 = self.yaw_error * 1.0
        self.yaw_output_1 = self.yaw_output * 1.0
        
        if abs(self.x_output) > self.max_vel:
            if self.x_output > 0:
                self.x_output = self.max_vel
            else:
                self.x_output = -self.max_vel

        if abs(self.y_output) > self.max_vel:
            if self.y_output > 0:
                self.y_output = self.max_vel
            else:
                self.y_output = -self.max_vel

        if abs(self.yaw_output) > self.max_vel_yaw:
            if self.yaw_output > 0:
                self.yaw_output = self.max_vel_yaw
            else:
                self.yaw_output = -self.max_vel_yaw

        if abs(self.z_output) > abs(self.max_vel_z):
            self.z_output = self.max_vel_z

        if abs(self.x_error) <= 0.01 and abs(self.y_error) <= 0.01 and abs(self.z_error) <= 0.24 and abs(yaw_error_deg) <= 2:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.z = 0.0
            if self.aruco_goal != 405:
                self.land_pub.publish(Empty())
                
            self.get_logger().info("Vehicle is centered with Aruco marker")
            raise ExitOk
            
        else:
            msg.linear.x = self.x_output
            msg.linear.y = self.y_output
            msg.linear.z = self.z_output
            msg.angular.z = self.yaw_output
                
        self.vel_pub.publish(msg)

    def conj_quat(self, q):
        conj = Quaternion()
        conj.w, conj.x, conj.y, conj.z = q.w, -q.x, -q.y, -q.z
        return conj
    
    def hammilton(self, q1, q2):
        product= Quaternion()

        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        product.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        product.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        product.y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        product.z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

        return product

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
        
    def quaternion_from_euler(self, roll=0, pitch=0, yaw=0):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        # q = numpy.array([0.0, 0.0, 0.0, 0.0])
        q = Quaternion()
        q.x = cy * cp * cr + sy * sp * sr
        q.y = cy * cp * sr - sy * sp * cr
        q.z = sy * cp * sr + cy * sp * cr
        q.w = sy * cp * cr - cy * sp * sr

        #q = q / numpy.linalg.norm(q)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoControl()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)