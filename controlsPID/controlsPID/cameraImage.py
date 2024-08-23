#!/usr/bin/env python3

import rclpy
import math
import time
import numpy as np
import cv2 as cv

from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

colorBajo1 = np.array([220, 49, 80], np.uint8)
colorAlto1 = np.array([250, 98, 86], np.uint8)

colorBajo2 = np.array([175, 100, 20], np.uint8)
colorAlto2 = np.array([179, 255, 255], np.uint8)

class LineDetect(Node):
    def __init__(self):
        super().__init__('line_dected_super')

        self.Bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera', self.listener_callback, 10)

    
    def listener_callback(self, data):
        current_frame = self.Bridge.imgmsg_to_cv2(data, "passthrough")
        new_current_frame = cv.resize(current_frame, (800, 600))
        cv.imshow("camera", new_current_frame)
        ################## Color Detection ######################
        frameHSV = cv.cvtColor(current_frame, cv.COLOR_BGR2HSV)
        maskColor1 = cv.inRange(frameHSV, colorBajo1, colorAlto1)
        #maskColor2 = cv.inRange(frameHSV, colorBajo2, colorAlto2)
        #maskColor = cv.add(maskColor1, maskColor2)

        maskColor_print = cv.bitwise_and(current_frame, current_frame, mask = maskColor1)
        new_mask_frame = cv.resize(maskColor_print, (800, 600))
        cv.imshow('maskColor_print', new_mask_frame)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    line_detect = LineDetect()
    rclpy.spin(line_detect)


    line_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




                 