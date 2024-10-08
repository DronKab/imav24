import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
import time
from smach import State

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

# Code for making the node runnable on Smach
class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self, id=105, red_flag=False):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.id = id
        self.red_follower = red_flag
    def execute(self, userdata):
        try:

            node = LineFollower(id_aruco=self.id, red_line_follower=self.red_follower)

            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except Exception as e:
            print(e)
            return "aborted"

class ControlsPID_Indoor():
    def __init__(self):

        self.node_rate = 10
        self.node_dt = 1 / self.node_rate

        self.signYaw = 0
        self.signPitch = 0
        self.signRoll = 0

        Kp_Yaw = 0.020
        Ki_Yaw = 0.0005
        Kd_Yaw = 0.003

        Kp_Pitch = 0.000
        Ki_Pitch = 0.000
        Kd_Pitch = 0.000

        Kp_Roll = 0.001
        Ki_Roll = 0.000
        Kd_Roll = 0.000

        Ts = 0.07

        self.A_Yaw = Kp_Yaw + Kd_Yaw / Ts + (Ki_Yaw * Ts) / 2
        self.B_Yaw = (Ki_Yaw * Ts) / 2 - Kp_Yaw - (2 * Kd_Yaw) / Ts
        self.C_Yaw = Kd_Yaw / Ts

        self.A_Pitch = Kp_Pitch + Kd_Pitch / Ts + (Ki_Pitch * Ts) / 2
        self.B_Pitch = (Ki_Pitch * Ts) / 2 - Kp_Pitch - (2 * Kd_Pitch) / Ts
        self.C_Pitch = Kd_Pitch / Ts 

        self.A_Roll = Kp_Roll + Kd_Roll / Ts + (Ki_Roll * Ts) / 2
        self.B_Roll = (Ki_Roll * Ts) / 2 - Kp_Roll - (2 * Kd_Roll) / Ts
        self.C_Roll = Kd_Roll / Ts

        self.U_yaw = [0.0, 0.0]
        self.Error_yaw = [0.0, 0.0, 0.0]

        self.U_pitch = [0.0, 0.0]
        self.Error_pitch = [0.0, 0.0, 0.0]

        self.U_roll = [0.0, 0.0]
        self.Error_roll = [0.0, 0.0, 0.0]


    def ControlPID_yaw(self, reference, Error_Actual_Total, High_limit, Low_limit):

        print(f"Error actual YAW: {Error_Actual_Total}")

        if Error_Actual_Total - reference >= 0.0:
            self.signYaw = 1.0
        
        else:
            self.signYaw = -1.0

        self.Error_yaw[0] = abs(Error_Actual_Total - reference)

        self.U_yaw[0] = self.U_yaw[1] + self.A_Yaw * self.Error_yaw[0] + self.B_Yaw * self.Error_yaw[1] + self.C_Yaw * self.Error_yaw[2]    
        
        if  self.U_yaw[0] > High_limit:
            self.U_yaw[0] = High_limit
        
        elif self.U_yaw[0] < Low_limit:
            self.U_yaw[0] = Low_limit

        inputControl = self.signYaw * self.U_yaw[0] 

        self.U_yaw[1] = self.U_yaw[0]

        self.Error_yaw[2] = self.Error_yaw[1]
        self.Error_yaw[1] = self.Error_yaw[0]

        return inputControl
    

    def ControlPID_pitch(self, reference, Error_Actual_Total, High_limit, Low_limit):

        #print(f"Error actual Total: {Error_Actual_Total}")

        if Error_Actual_Total - reference >= 0.0:
            self.signPitch = 1.0
        
        else:
            self.signPitch = -1.0

        self.Error_pitch[0] = abs(Error_Actual_Total - reference)

        self.U_pitch[0] = self.U_pitch[1] + self.A_Pitch * self.Error_pitch[0] + self.B_Pitch * self.Error_pitch[1] + self.C_Pitch * self.Error_pitch[2]

        if self.U_pitch[0] > High_limit:
            self.U_pitch[0] = High_limit

        elif self.U_pitch[0] < Low_limit:
            self.U_pitch[0] = Low_limit

        inputControl = self.signPitch * self.U_pitch[0]

        self.U_pitch[1] = self.U_pitch[0]

        self.Error_pitch[2] = self.Error_pitch[1]
        self.Error_pitch[1] = self.Error_pitch[0]

        return inputControl

    
    def ControlPID_roll(self, reference, Error_Actual_Total, High_limit, Low_limit):
        
        #print(f"Error actual Total: {Error_Actual_Total}")

        if Error_Actual_Total - reference >= 0.0:
            self.signRoll = 1.0
        
        else:
            self.signRoll = -1.0

        self.Error_roll[0] = abs(Error_Actual_Total - reference)

        self.U_roll[0] = self.U_roll[1] + self.A_Roll * self.Error_roll[0] + self.B_Roll * self.Error_roll[1] + self.C_Roll * self.Error_roll[2]

        if self.U_roll[0] > High_limit:
            self.U_roll[0] = High_limit

        elif self.U_roll[0] < Low_limit:
            self.U_roll[0] = Low_limit

        inputControl = self.signRoll * self.U_roll[0]

        self.U_roll[1] = self.U_roll[0]

        self.Error_roll[2] = self.Error_roll[1]
        self.Error_roll[1] = self.Error_roll[0]

        return inputControl

class LineFollower(Node):
    def __init__(self, id_aruco=105, red_line_follower=False):
        super().__init__('line_follower')
        self.get_logger().info("Line Follower started.")

        # Node Variables 
        # Main Follower
        # Blue Color in Simulation
        #self.colorBajo1 = np.array([110, 255, 255], np.uint8)
        #self.colorAlto1 = np.array([130, 255, 255], np.uint8)

        # Blue Color Real Life
        self.colorBajo1 = np.array([88, 86, 96], np.uint8)
        self.colorAlto1 = np.array([105, 209, 151], np.uint8)

        # Red Color Real Life
        self.lower_red1 = np.array([0, 64, 100])
        self.upper_red1 = np.array([16, 221, 166])

        self.lower_red2 = np.array([168, 64, 100])
        self.upper_red2 = np.array([180, 221, 166])

        # Second Follower
        self.red_line = red_line_follower
        # Red Color in Simulation
        #self.lower_red1 = np.array([0, 50, 50])
        #self.upper_red1 = np.array([10, 255, 255])

        #self.lower_red2 = np.array([170, 50, 50])
        #self.upper_red2 = np.array([180, 255, 255])

        self.cv_bridge = CvBridge()

        self.vels = Twist()

        self.angle_error = 0
        self.lateral_error = 0
        self.p_yaw = 0.1
        self.control_flag = 0

        self.Control = ControlsPID_Indoor()

        self.aruco_fin = id_aruco
        self.aruco_flag = 0

        # Subscriptions
        self.aruco_subs = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 10)
        self.image_sub = self.create_subscription(CompressedImage, '/pi_camera/image_raw/compressed', self.image_callback, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, "/px4_driver/cmd_vel", 10)
        self.debug_pub = self.create_publisher(CompressedImage, "/line_follower/debug/compressed", 10)
        
        # Timer to publish control
        self.ts = 0.07

        # Flag from Red to Blue
        self.turn_in_red_line = False

    def aruco_callback(self, msg):
        if len(msg.markers) > 0:
            for i in range (0, len(msg.markers)):
                if msg.markers[i].marker_id == self.aruco_fin:
                    self.aruco_flag = 1
                    break


    def image_callback(self, msg):

        if self.aruco_flag == 1:
            self.aruco_flag = 0
            raise ExitOk
        else:

            start_time = time.time()

            # Initialize variables to track the minimum angle error and the corresponding contour
            #min_angle_error = float('inf')
            #min_angle_contour = None

            # Initialize min_angle_error and min_lateral_error to ensure they're always defined
            min_angle_error = 0
            min_lateral_error = 0

            # Initialize variables to track the minimum center Y position and the corresponding contour
            min_centery_line = float('inf')
            max_angle_error = 30.0

            # Line Detection
            frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            height_scr, width_scr, _ = frame.shape
            center_camera_x = width_scr * 0.5
            center_camera_y = height_scr * 0.5

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.line(frame, (int(center_camera_x + 40), 0), (int(center_camera_x + 40), height_scr), (0, 0, 0), 10)
            cv2.line(frame, (int(center_camera_x - 40), 0), (int(center_camera_x - 40), height_scr), (0, 0, 0), 10)

            cv2.line(frame, (0, int(center_camera_y + 40)), (width_scr, int(center_camera_y + 40)), (0, 0, 0), 10)
            cv2.line(frame, (0, int(center_camera_y - 40)), (width_scr, int(center_camera_y - 40)), (0, 0, 0), 10)

            if self.turn_in_red_line == True:
                blue_frame = cv2.inRange(frame, self.colorBajo1, self.colorAlto1)
                blue_mask = blue_frame
                blue_frame = cv2.GaussianBlur(blue_frame, (5, 5), 0)
                blue_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                # Aplicar operación de apertura (erosión seguida de dilatación) si quieres eliminar ruido puntual
                blue_frame = cv2.morphologyEx(blue_frame, cv2.MORPH_OPEN, blue_kernel)

                # Aplicar operación de cierre (dilatación seguida de erosión) si deseas cerrar pequeños agujeros en los objetos detectados
                blue_frame = cv2.morphologyEx(blue_frame, cv2.MORPH_CLOSE, blue_kernel) 
                blue_mask = blue_frame
                blue_debug_img = cv2.cvtColor(blue_frame, cv2.COLOR_GRAY2BGR)
                blue_contours, blue_hierarchy = cv2.findContours(blue_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(blue_contours) > 0:
                    self.vels.angular.z = 0.5
                    self.vels.linear.x = 0.0
                    self.vels.linear.y = 0.0
                    self.vel_pub.publish(self.vels)
                    time.sleep(0.1)
                    self.vel_pub.publish(self.vels)
                    time.sleep(0.1)
                    self.vel_pub.publish(self.vels)
                    time.sleep(0.1)
                    raise ExitOk

            if self.red_line == True:
                maskRed1 = cv2.inRange(frame, self.lower_red1, self.upper_red1)
                maskRed2 = cv2.inRange(frame, self.lower_red2, self.upper_red2)
                red_frame = maskRed1 | maskRed2
                red_frame = cv2.GaussianBlur(red_frame, (5, 5), 0)
                red_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                red_frame = cv2.morphologyEx(red_frame, cv2.MORPH_OPEN, red_kernel)
                red_frame = cv2.morphologyEx(red_frame, cv2.MORPH_CLOSE, red_kernel) 
                red_mask = red_frame
                red_debug_img = cv2.cvtColor(red_frame, cv2.COLOR_GRAY2BGR)
                red_contours, red_hierarchy = cv2.findContours(red_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(red_contours) > 0:
                    debug_img = red_debug_img
                    contours = red_contours
                    mask = red_mask
                else:
                    frame = cv2.inRange(frame, self.colorBajo1, self.colorAlto1)
                    mask = frame
                    frame = cv2.GaussianBlur(frame, (5, 5), 0)
                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                    # Aplicar operación de apertura (erosión seguida de dilatación) si quieres eliminar ruido puntual
                    frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)

                    # Aplicar operación de cierre (dilatación seguida de erosión) si deseas cerrar pequeños agujeros en los objetos detectados
                    frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel) 
                    mask = frame

                    debug_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            else:
                
                frame = cv2.inRange(frame, self.colorBajo1, self.colorAlto1)
                mask = frame
                frame = cv2.GaussianBlur(frame, (5, 5), 0)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                # Aplicar operación de apertura (erosión seguida de dilatación) si quieres eliminar ruido puntual
                frame = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)

                # Aplicar operación de cierre (dilatación seguida de erosión) si deseas cerrar pequeños agujeros en los objetos detectados
                frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel) 
                mask = frame

                debug_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


            if len(contours)>0:
                for contour in contours:
                    ((centerx_line, centery_line), (height, width), angle) = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(((centerx_line, centery_line), (height, width), angle))
                    box = np.intp(box)

                    cv2.putText(debug_img, f'AngleNoMod : {angle}', (20,60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    
                    if height > width:
                        angle -= 90
                    
                    # Calculate angle error
                    general_angle_error = angle
                    general_lateral_error = width_scr / 2 - centerx_line
                    
                    # Check if this is the smallest angle error encountered
                    """if abs(self.angle_error) < abs(min_angle_error):
                        min_angle_error = self.angle_error
                        min_angle_contour = contour
                        min_angle_box = box
                        min_centerx_line = centerx_line
                        min_centery_line = centery_line
                        min_lateral_error = self.lateral_error"""
                    
                    #Check if this is the smallest center Y line position
                    if centery_line < min_centery_line:
                        self.angle_error = general_angle_error
                        min_centery_line = centery_line
                        min_angle_contour = contour
                        min_angle_box = box
                        min_centerx_line = centerx_line
                        self.lateral_error = general_lateral_error
                    
                # If a contour with minimum angle error was found, process it
                if min_angle_contour is not None:

                    # Draw lines for frame division
                    cv2.line(debug_img, (int(center_camera_x - 20), 0), (int(center_camera_x - 20), height_scr), (0, 255, 0), 2)
                    cv2.line(debug_img, (int(center_camera_x + 20), 0), (int(center_camera_x + 20), height_scr), (0, 255, 0), 2)

                    # Calculate area of the contour in each division
                    mask_contour = np.zeros_like(mask)
                    cv2.drawContours(mask_contour, [min_angle_contour], -1, 255, thickness=cv2.FILLED)
                    area_total = cv2.countNonZero(mask_contour)
                    area_left = cv2.countNonZero(mask_contour[:, :int(center_camera_x - 20)])
                    area_right = cv2.countNonZero(mask_contour[:, int(center_camera_x + 20):])

                    # Mostrar áreas en pantalla
                    cv2.putText(debug_img, f'Area Left: {area_left}', (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    cv2.putText(debug_img, f'Area Right: {area_right}', (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    cv2.putText(debug_img, f'Total Area: {area_total}', (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)


                    # Highlight the contour with the minimum angle error
                    cv2.drawContours(debug_img, [min_angle_contour], -1, (255, 0, 0), 5)
                    cv2.drawContours(debug_img, [min_angle_box], 0, (0, 0, 255), 2)

                    # Use the lateral error and angle error for control (e.g., PID control)
                        #Condition for turn around

                    if self.angle_error >= max_angle_error or self.angle_error <= ((-1.0)*max_angle_error):
                        self.control_flag = 0
                        print("BIG ANGLE")
                        if area_right > area_left:
                            print("TURN RIGHT")
                            cv2.putText(debug_img, 'Turn Right', (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                            self.angle_error = 0
                            self.vels.angular.z = 1.0
                        else:
                            print("TURN LEFT")
                            if self.red_line == True:
                                self.turn_in_red_line = True
                            cv2.putText(debug_img, 'Turn Left', (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                            self.angle_error = 0
                            self.vels.angular.z = -1.0

                        self.vels.linear.x = 0.1
                        self.vels.linear.y = 0.0
                    else:
                        self.control_flag = 1
                        self.vels.linear.x = 0.3
                        self.vels.linear.y = float(self.Control.ControlPID_roll(0, self.lateral_error, 1, 0))
                        self.vels.angular.z = float(self.Control.ControlPID_yaw(0, self.angle_error, 1, 0))

                    # Annotate the image with the error values of the selected contour
                    """cv2.putText(debug_img, f'Min AngleError: {min_angle_error}', 
                                (int(min_centerx_line), int(min_centery_line) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)"""
                    
                    cv2.putText(debug_img, f'AngleError: {self.angle_error}', 
                                (int(min_centerx_line), int(min_centery_line) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    cv2.putText(debug_img, f'LateralError: {self.lateral_error}', 
                                (int(min_centerx_line), int(min_centery_line) + 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                    print(f"Front Velocity: {self.vels.linear.x}")
                    print(f"Velocity Roll: {self.vels.linear.y}         Angular velocity: {self.vels.angular.z}")
                    self.vel_pub.publish(self.vels)
                else:
                    print("NO HAY CONTORNO MINIMO")
                    cv2.putText(debug_img, 'NO HAY CONTORNO MINIMO', (int(width_scr/2),int(height_scr/2+20)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 255), 1)
                    self.angle_error = 0
                    self.lateral_error = 0    
        
            else:
                print("NO HAY CONTORNOS")
                cv2.putText(debug_img, 'NO HAY CONTORNOS', (int(width_scr/2), int(height_scr/2)), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 255), 1)
                self.angle_error = 0
                self.lateral_error = 0
                # If thers isn't contours, all velocities are in 0.
                self.vels.angular.z = 0.0
                self.vels.linear.x = 0.0
                self.vels.linear.y = 0.0
                self.vel_pub.publish(self.vels)
                

            cv2.putText(debug_img, f'AngleError : {min_angle_error}', (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            cv2.putText(debug_img, f'LateralError : {min_lateral_error}', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

            # Image Publisher  
            resized_image = cv2.resize(debug_img, (200, 100), interpolation=cv2.INTER_LINEAR)
            debug_msg = self.cv_bridge.cv2_to_compressed_imgmsg(resized_image)
            self.debug_pub.publish(debug_msg)

            # Mostrar la imagen de detección
            #cv2.imshow('Detection', debug_img)
            cv2.waitKey(1)

            end_time = time.time()
            execution_time = end_time - start_time
            print(f"Tiempo de ejecución del callback: {execution_time:.4f} segundos")


def main(args=None):
    rclpy.init(args=args)
    line_follower= LineFollower()
    rclpy.spin(line_follower)

    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)