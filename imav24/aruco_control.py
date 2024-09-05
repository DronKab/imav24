import rclpy
import numpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
from aruco_opencv_msgs.msg import ArucoDetection

class ArucoControl(Node):
    def __init__(self):
        super().__init__("aruco_control")
        self.get_logger().info("Started Aruco Control ...")

        self.max_vel = 4.0
        self.max_vel_z = -0.1

        self.last_known_x = None
        self.last_known_y = None
        self.last_known_z = None
        self.aruco_visible = False

        # Node variables 
        self.aruco_id = 0
        # self.aruco_goal = 300 # first platform
        # self.aruco_goal = 301 # second platform
        # self.aruco_goal = 302 # third platform
        # self.aruco_goal = 400 # cone collection
        self.aruco_goal = 405 # cone placement 

        self.aruco_sub = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)

        self.vel_pub = self.create_publisher(Twist, "/px4_driver/cmd_vel", 10)
        self.land_pub = self.create_publisher(Empty, "/px4_driver/land", 10)
        self.do_height_control_pub = self.create_publisher(Bool, "/px4_driver/do_height_control", 10)

        if (self.aruco_goal == 300 or self.aruco_goal == 301 or self.aruco_goal == 302):
            self.declare_parameter("do_height_control", False)
            self.x_distance = 0.0
            self.y_distance = 0.0

            # X variables
            self.declare_parameter("px_gain", 0.9)
            self.px_gain = self.get_parameter("px_gain").get_parameter_value().double_value
            self.declare_parameter("dx_gain", 0.1)
            self.dx_gain = self.get_parameter("dx_gain").get_parameter_value().double_value
            self.declare_parameter("nx_filter", 0.1)
            self.nx_filter = self.get_parameter("nx_filter").get_parameter_value().double_value

            # Y variables
            self.declare_parameter("py_gain", 0.9)
            self.py_gain = self.get_parameter("py_gain").get_parameter_value().double_value
            self.declare_parameter("dy_gain", 0.1)
            self.dy_gain = self.get_parameter("dy_gain").get_parameter_value().double_value
            self.declare_parameter("ny_filter", 0.1)
            self.ny_filter = self.get_parameter("ny_filter").get_parameter_value().double_value

            # Z variables
            self.declare_parameter("pz_gain", 0.6)
            self.pz_gain = self.get_parameter("pz_gain").get_parameter_value().double_value
            self.declare_parameter("dz_gain", 0.1)
            self.dz_gain = self.get_parameter("dz_gain").get_parameter_value().double_value
            self.declare_parameter("nz_filter", 0.1)
            self.nz_filter = self.get_parameter("nz_filter").get_parameter_value().double_value

        else:
            if self.aruco_goal == 400:
                self.declare_parameter("do_height_control", False)
                self.x_distance = 0.4
                self.y_distance = 0.0

            else:
                self.declare_parameter("do_height_control", True)
                self.x_distance = 0.0
                self.y_distance = 0.0

            # X variables
            self.declare_parameter("px_gain", 0.5)
            self.px_gain = self.get_parameter("px_gain").get_parameter_value().double_value
            self.declare_parameter("dx_gain", 0.06)
            self.dx_gain = self.get_parameter("dx_gain").get_parameter_value().double_value
            self.declare_parameter("nx_filter", 0.1)
            self.nx_filter = self.get_parameter("nx_filter").get_parameter_value().double_value

            # Y variables 
            self.declare_parameter("py_gain", 0.5)
            self.py_gain = self.get_parameter("py_gain").get_parameter_value().double_value
            self.declare_parameter("dy_gain", 0.06)
            self.dy_gain = self.get_parameter("dy_gain").get_parameter_value().double_value
            self.declare_parameter("ny_filter", 0.1)
            self.ny_filter = self.get_parameter("ny_filter").get_parameter_value().double_value

            # Z variables #
            self.declare_parameter("pz_gain", 0.3)
            self.pz_gain = self.get_parameter("pz_gain").get_parameter_value().double_value
            self.declare_parameter("dz_gain", 0.03)
            self.dz_gain = self.get_parameter("dz_gain").get_parameter_value().double_value
            self.declare_parameter("nz_filter", 0.1)
            self.nz_filter = self.get_parameter("nz_filter").get_parameter_value().double_value
        
        self.get_logger().info(f"PX Gain : {self.px_gain}")
        self.get_logger().info(f"DX Gain : {self.dx_gain}")
        self.get_logger().info(f"NX Coefficient : {self.nx_filter}")
        self.get_logger().info(f"PY Gain : {self.py_gain}")
        self.get_logger().info(f"DY Gain : {self.dy_gain}")
        self.get_logger().info(f"NY Coefficient : {self.ny_filter}")
        self.get_logger().info(f"PZ Gain : {self.pz_gain}")
        self.get_logger().info(f"DZ Gain : {self.dz_gain}")
        self.get_logger().info(f"NZ Coefficient : {self.nz_filter}")
        
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

        self.do_height_control = self.get_parameter("do_height_control").get_parameter_value().bool_value

        # self.node_rate = 15.625 # 1/64ms
        # self.ts = 1 / self.node_rate
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

        if not self.aruco_visible and self.last_known_x is not None:
            self.x_error = self.last_known_x * 0.1
            self.y_error = self.last_known_y * 0.1
            self.z_error = self.last_known_z * 0.01

        do_height_control_msg = Bool()
        do_height_control_msg.data = self.do_height_control
        self.do_height_control_pub.publish(do_height_control_msg)

        # Crear el mensaje de velocidad
        self.get_logger().info("Entering control")
        self.get_logger().info(f"Erores: x={self.x_error}, y={self.y_error},  z={self.z_error}")

        # Control PD en x
        px_action = self.x_error * self.px_gain
        # ix_action = self.x_output_1 + self.x_error * self.ix_gain * self.ts
        dx_action = self.x_output_1 * (self.nx_filter * self.dx_gain * (self.x_error - self.x_error_1)) / (1 + self.nx_filter * self.ts)
        # self.x_output = float(px_action + ix_action + dx_action)
        self.x_output = float(px_action + dx_action)
        self.x_error_1 = self.x_error * 1.0
        self.x_output_1 = self.x_output * 1.0
        
        # Control PD en Y
        py_action = self.y_error * self.py_gain
        # iy_action = self.y_output_1 + self.y_error * self.iy_gain * self.ts
        dy_action = self.y_output_1 * (self.ny_filter * self.dy_gain * (self.y_error - self.y_error_1)) / (1 + self.ny_filter * self.ts)
        # self.y_output = float(py_action + iy_action + dy_action)
        self.y_output = float(py_action + dy_action)
        self.y_error_1 = self.y_error * 1.0
        self.y_output_1 = self.y_output * 1.0

        # Control PD en Z
        pz_action = self.z_error * self.pz_gain
        # iz_action = self.z_output_1 + self.z_error * self.iz_gain * self.ts
        dz_action = self.z_output_1 * (self.nz_filter * self.dz_gain * (self.z_error - self.z_error_1)) / (1 + self.nz_filter * self.ts)
        self.z_output = float(pz_action + dz_action)
        # self.z_output = float(pz_action + iz_action + dz_action)
        self.z_error_1 = self.z_error * 1.0
        self.z_output_1 = self.y_output * 1.0
        
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

        if abs(self.z_output) > abs(self.max_vel_z):
            self.z_output = self.max_vel_z

        if abs(self.x_error) <= 0.01 and abs(self.y_error) <= 0.01 and abs(self.z_error) <= 0.24:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            if self.aruco_goal != 405:
                self.land_pub.publish(Empty())
                
            self.get_logger().info("Vehicle is centered with Aruco marker")
            

        else:
            msg.linear.x = self.x_output
            msg.linear.y = self.y_output
            msg.linear.z = self.z_output
            # if self.aruco_visible:
                # msg.linear.z = self.z_output
            # else:
                # msg.linear.x = self.x_output * 0.5
                # msg.linear.y = self.y_output * 0.5
                # msg.linear.z = self.z_output * 0.5
                
            
        self.vel_pub.publish(msg)

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