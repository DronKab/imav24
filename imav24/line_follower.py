import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info("Line Follower started.")

        # Node Variables 
        self.colorBajo1 = np.array([110, 255, 255], np.uint8)
        self.colorAlto1 = np.array([130, 255, 255], np.uint8)
        self.cv_bridge = CvBridge()

        self.angle_error = 0
        self.lateral_error = 0
        self.p_yaw = 0.1

        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/pi_camera/image_raw', self.image_callback, 10)

        # Publishers
        self.vel_pub = self.create_publisher(Twist, "/px4_driver/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/line_follower/debug", 10)

        # Timer to publish control
        self.ts = 0.1
        self.heartbeat_timer = self.create_timer( self.ts, self.yaw_control)

    def image_callback(self, msg):
        # Line Detection
        frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        height_scr, width_scr, _ = frame.shape

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame = cv2.inRange(frame, self.colorBajo1, self.colorAlto1)
        kernel = np.ones((5, 5), np.uint8) 
        frame = cv2.dilate(frame, kernel, iterations=15) 

        debug_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)>0:
            max_contour = max(contours, key=cv2.contourArea)
            ((centerx_line, centery_line),(height, width),angle) = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(((centerx_line, centery_line),(height, width),angle))
            box = np.int0(box)
            if height > width:
                angle -= 90

            self.angle_error = angle
            self.lateral_error = width_scr/2 - centerx_line

            cv2.drawContours(debug_img, max_contour, -1, (255, 0, 0), 5)
            cv2.drawContours(debug_img,[box],0,(0,0,255),2)
        else:
            self.angle_error = 0
            self.lateral_error = 0

        cv2.putText(debug_img, f'AngleError : {self.angle_error}', (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        cv2.putText(debug_img, f'LateralError : {self.lateral_error}', (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.debug_pub.publish(debug_msg)

    def yaw_control(self):
        msg = Twist()
        msg.angular.z = float(self.p_yaw * self.angle_error)
        self.vel_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    line_follower= LineFollower()
    rclpy.spin(line_follower)

    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()