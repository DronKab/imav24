#!/usr/bin/env python3

import rclpy
import math
import time
import numpy as np
import cv2 as cv
from smach import State

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs. msg import Twist

from cv_bridge import CvBridge

# Define the range for red color in HSV
# Lower and upper bounds for red color
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

# Code for making the node runnable on Smach
class ExitOk(Exception): pass
class NodeState(State):
    def __init__(self):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.id = id
    def execute(self, userdata):
        try:

            node = TakePhoto()

            rclpy.spin(node)
        except ExitOk:
            node.destroy_node()
            return "succeeded"
        except:
            return "aborted"

class TakePhoto(Node):
    def __init__(self):
        super().__init__('Photo')

        self.Bridge = CvBridge()

        self.buffer_size = 5
        self.x_buffer = []
        self.y_buffer = []

        self.errorX = 0
        self.errorY = 0

        self.error_threshold = 60

        self.image_taken = False

        #Subscriptions
        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10)

    def listener_callback(self, data):
        current_frame = self.Bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        timestamp = int(time.time())
        filename = f"image_{timestamp}.png"
        cv.imwrite(filename, current_frame)
        self.get_logger().info(f"Imagen guardada como {filename}")

        self.image_taken = True
        if self.image_taken == True:
            raise ExitOk
            cv.destroyAllWindows()

            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    Photo = TakePhoto()
    rclpy.spin(Photo)
    Photo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
