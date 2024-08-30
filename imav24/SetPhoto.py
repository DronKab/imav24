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


# Define the range for red color in HSV
# Lower and upper bounds for red color
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])


class TakePhoto(Node):
    def __init__(self):
        super().__init__('Photo')

        self.Bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, 10)


    def listener_callback(self, data):
        current_frame = self.Bridge.imgmsg_to_cv2(data, "bgr8")
        resized_frame = cv.resize(current_frame, (620, 480))
        center_x, center_y = resized_frame.shape[1] // 2, resized_frame.shape[0] // 2
        top_left_x, top_left_y = center_x - 50, center_y - 50
        bottom_right_x, bottom_right_y = center_x + 50, center_y + 50
        
        frame_draw = resized_frame.copy()
        cv.circle(frame_draw, (center_x, center_y), 15, (0, 0, 255), -1)
        cv.rectangle(frame_draw, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (255, 0, 0), 2) 
        
        frameHSV = cv.cvtColor(resized_frame, cv.COLOR_BGR2HSV)
        maskColor1 = cv.inRange(frameHSV, lower_red1, upper_red1)
        maskColor2 = cv.inRange(frameHSV, lower_red2, upper_red2)

        red_mask = maskColor1 | maskColor2

        result = cv.bitwise_and(resized_frame, resized_frame, mask=red_mask)

        if len(red_mask.shape) == 2:
            red_mask_colored = cv.cvtColor(red_mask, cv.COLOR_GRAY2BGR)
        else:
            red_mask_colored = red_mask

        if len(result.shape) == 2:
            result_colored = cv.cvtColor(result, cv.COLOR_GRAY2BGR)
        else:
            result_colored = result

        top_row = np.hstack((resized_frame, frame_draw))
        bottom_row = np.hstack((red_mask_colored, result_colored))
        final_image = np.vstack((top_row, bottom_row))

        final_image = np.vstack((top_row, bottom_row))

        cv.imshow("Composite Image", final_image)
        cv.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    Photo = TakePhoto()
    rclpy.spin(Photo)
    Photo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

