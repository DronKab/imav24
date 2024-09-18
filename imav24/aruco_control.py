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
    def __init__(self, id=105, dx=0.0, dy=0.0, yaw_tg=180, flg_lnd=1):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.id = id
        self.dx = dx
        self.dy = dy
        self.yaw_tg = yaw_tg
        self.flg_ln = flg_lnd
    def execute(self, userdata):
        try:
            node = ArucoControl(id_gl=self.id, dst_x=self.dx, dst_y=self.dy, yaw_trg=self.yaw_tg, flg_land=self.flg_ln)
            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except Exception as e:
            print(e)
            return "aborted"

class ArucoControl(Node):
    def __init__(self, id_gl=105, dst_x=0.0, dst_y=0.0, yaw_trg=180, flg_land=1):
        super().__init__("aruco_control")
        self.get_logger().info("Started Aruco Control ...")

        self.max_vel = 0.5
        self.max_vel_z = -0.1
        self.max_vel_yaw = 10.0

        self.last_known_x = None
        self.last_known_y = None
        self.last_known_z = None
        self.last_known_yaw = None
        self.aruco_visible = False

        # Received parameters
        self.aruco_goal = id_gl
        self.x_distance = dst_x
        self.y_distance = dst_y
        self.yaw = yaw_trg
        self.flag_land = flg_land

        self.lim_sup_x = 0.1
        self.lim_sup_y = 0.1
        self.lim_sup_yaw = 15
        self.lim_inf_yaw = 10
        self.lim_z = 0.5
        self.lim_inf_x = 0.05
        self.lim_inf_y = 0.05

        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)

        self.vel_pub = self.create_publisher(Twist, "/px4_driver/cmd_vel", 10)
        self.land_pub = self.create_publisher(Empty, "/px4_driver/land", 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        if self.flag_land == 1:
            self.declare_parameter("do_height_control", False)

        else:
            self.declare_parameter("do_height_control", True)
        
        # X variables
        self.px_gain = 0.3
        self.dx_gain = 0.03
        self.nx_filter = 0.03

        # Y variables
        self.py_gain = 0.3
        self.dy_gain = 0.03
        self.ny_filter = 0.03

        # Z variables
        self.pz_gain = 0.2
        self.dz_gain = 0.03
        self.nz_filter = 0.1

        # Yaw variables
        self.pyaw_gain = 0.01
        # self.dyaw_gain = 0.00
        # self.nyaw_filter = 0.0
        
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
                    aruco_index = i
                    self.get_logger().info(f"Aruco ID: {msg.markers[i].marker_id}")
                    break

            if msg.markers[aruco_index].marker_id == self.aruco_goal:
                self.aruco_id = msg.markers[aruco_index].marker_id
                self.q1 = msg.markers[aruco_index].pose.orientation
                _, _, angle_aruco = self.euler_from_quaternion(self.q1)
                angle_aruco = math.degrees(angle_aruco)

                if math.isnan(angle_aruco):
                    self.yaw_error = self.last_known_yaw
                else:
                    self.yaw_error = angle_aruco - self.yaw

                if math.isnan(msg.markers[aruco_index].pose.position.y):
                    self.x_error = self.last_known_x
                else:
                    self.x_error = -msg.markers[aruco_index].pose.position.y + self.x_distance
                
                if math.isnan(msg.markers[aruco_index].pose.position.x):
                    self.y_error = self.last_known_y
                else:
                    self.y_error = -msg.markers[aruco_index].pose.position.x + self.y_distance
                
                if math.isnan(msg.markers[aruco_index].pose.position.z):
                    self.z_error = self.last_known_z
                else:
                    self.z_error = -msg.markers[aruco_index].pose.position.z
                
                if abs(self.yaw_error) > 180:
                    if self.yaw_error < 0:
                        self.yaw_error = self.yaw_error + 360 
                    else:    
                        self.yaw_error = self.yaw_error - 360

                self.last_known_x = self.x_error
                self.last_known_y = self.y_error
                self.last_known_z = self.z_error
                self.last_known_yaw = self.yaw_error
                self.aruco_visible = True 

            else:
                self.aruco_visible = False

        else:
            self.aruco_visible = False


    def control(self):
        msg = Twist()
        if not self.aruco_visible and self.last_known_x is not None:
            self.x_error = self.last_known_x * 0.2
            self.y_error = self.last_known_y * 0.2
            self.z_error = self.last_known_z * 0.001
            self.yaw_error = self.last_known_yaw 

        self.get_logger().info("Entering control")
        self.get_logger().info(f"Erores: x={self.x_error}, y={self.y_error},  z={self.z_error}, yaw={self.yaw_error}")

        # Control PD en X y Y
        if abs(self.yaw_error) < self.lim_sup_yaw:
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
            self.x_error = 0.0
            self.x_output = 0.0
            self.y_error = 0.0
            self.y_output = 0.0
            self.z_error = 0.0
            self.z_output = 0.0

        
        # Control PD en z
        if self.flag_land == 1:
            if abs(self.x_error) < self.lim_sup_x and abs(self.y_error) < self.lim_sup_y and abs(self.yaw_error) < self.lim_sup_yaw:
                # Control PD en Z
                pz_action = self.z_error * self.pz_gain
                # iz_action = self.z_output_1 + self.z_error * self.iz_gain * self.ts
                dz_action = self.z_output_1 * (self.nz_filter * self.dz_gain * (self.z_error - self.z_error_1)) / (1 + self.nz_filter * self.ts)
                self.z_output = float(pz_action + dz_action)
                # self.z_output = float(pz_action + iz_action + dz_action)
            else:
                self.z_error = 0.0
                self.z_output = 0.0
        
        self.z_error_1 = self.z_error * 1.0
        self.z_output_1 = self.z_output * 1.0

        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        if abs(self.yaw_error) > self.lim_inf_yaw:
            # Control PD en Yaw
            pyaw_action = self.yaw_error * self.pyaw_gain
            # iyaw_action = self.yaw_output_1 + self.yaw_error * self.iyaw_gain * self.ts
            # dyaw_action = self.yaw_output_1 * (self.nyaw_filter * self.dyaw_gain * (self.yaw_error - self.yaw_error_1)) / (1 + self.nyaw_filter * self.ts)
            # self.yaw_output = float(pyaw_action + iyaw_action + dyaw_action)
            self.yaw_output = float(pyaw_action)
            self.yaw_error_1 = self.yaw_error * 1.0
            self.yaw_output_1 = self.yaw_output * 1.0
        
        else:
            self.yaw_output = 0.0
            self.yaw_error = 0.0
            self.yaw_error_1 = 0.0
            self.yaw_error_1 = 0.0
            self.yaw_output_1 = 0.0
        
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

        if self.flag_land == 1:
            if abs(self.x_error) <= self.lim_inf_x and abs(self.y_error) <= self.lim_inf_y and abs(self.z_error) <= self.lim_z and abs(self.yaw_error) <= self.lim_inf_yaw:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = 0.0
                self.land_pub.publish(Empty())
                self.get_logger().info("Vehicle is centered with Aruco marker")
                raise ExitOk

        else:
            if  abs(self.x_error) <= self.lim_inf_x and abs(self.y_error) <= self.lim_inf_y and abs(self.yaw_error) <= self.lim_inf_yaw:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.z = 0.0
                self.get_logger().info("Vehicle is centered with Aruco marker")
                raise ExitOk
        
        msg.linear.x = self.x_output
        msg.linear.y = self.y_output
        msg.linear.z = self.z_output
        msg.angular.z = self.yaw_output

        self.get_logger().info("Velocity published:")
        self.get_logger().info(f"x={self.x_output}, y={self.y_output},  z={self.z_output}, yaw={self.yaw_output}")
                
        self.vel_pub.publish(msg)

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