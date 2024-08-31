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

class ControlsPID_Indoor():
    def __init__(self):

        self.node_rate = 10
        self.node_dt = 1 / self.node_rate

        self.signYaw = 0
        self.signPitch = 0
        self.signRoll = 0

        Kp_Yaw = 0.08
        Ki_Yaw = 0.0002
        Kd_Yaw = 0.010

        Kp_Pitch = 0.000
        Ki_Pitch = 0.000
        Kd_Pitch = 0.000

        Kp_Roll = 0.010
        Ki_Roll = 0.000
        Kd_Roll = 0.000

        Ts = 0.1

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


class TakePhoto(Node):
    def __init__(self):
        super().__init__('Photo')

        self.Bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.listener_callback, 10)

        self.buffer_size = 5
        self.x_buffer = []
        self.y_buffer = []

        self.error_threshold = 40

        self.image_taken = False

    def moving_average(self, value, buffer, buffer_size):
        if len(buffer) >= buffer_size:
            buffer.pop(0)
        
        buffer.append(value)

        return np.mean(buffer) if buffer else 0

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

        contours, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        center_of_polygon = None
        error = None

        if contours:
            contour = max(contours, key=cv.contourArea)
            epsilon = 0.02 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:
                cv.drawContours(frame_draw, [approx], -1, (0, 255, 0), 2)

                M = cv.moments(contour)

                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])

                    cX_smoothed = self.moving_average(cX, self.x_buffer, self.buffer_size)
                    cY_smoothed = self.moving_average(cY, self.y_buffer, self.buffer_size)

                    center_of_polygon = (int(cX_smoothed), int(cY_smoothed))
                    cv.circle(frame_draw, (cX, cY), 10, (0, 0, 255), -1)

        if center_of_polygon:
            cv.line(frame_draw, (center_x, center_y), center_of_polygon, (0, 0, 255), 5)
            error = (center_of_polygon[0] - center_x, center_y - center_of_polygon[1])

            error_text = [
                f"Error X: {error[0]}",
                f"Error Y: {error[1]}"
            ]

            for i, line in enumerate(error_text):
                cv.putText(frame_draw, line, (10, 30 + i * 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 150), 2, cv.LINE_AA)

            if abs(error[0]) < self.error_threshold and abs(error[1]) < self.error_threshold:
                mask_polygon = np.zeros_like(red_mask)
                cv.drawContours(mask_polygon, [approx], -1, 255, thickness=cv.FILLED)

                polygon_roi = cv.bitwise_and(resized_frame, resized_frame, mask=mask_polygon)

                x, y, w, h = cv.boundingRect(mask_polygon)
                cropped_roi = polygon_roi[y:y+h, x:x+w]

                timestamp = int(time.time())
                filename = f"image_{timestamp}.png"
                cv.imwrite(filename, cropped_roi)
                self.get_logger().info(f"Imagen guardada como {filename}")

                self.image_taken = True

                cv.destroyAllWindows()

                self.destroy_node()
                rclpy.shutdown()

        else:
            error_text = [
                f"Error X: None",
                f"Error Y: None"
            ]

            for i, line in enumerate(error_text):
                cv.putText(frame_draw, line, (10, 30 + i * 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 150), 2, cv.LINE_AA)

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

