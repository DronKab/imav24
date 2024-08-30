#!/usr/bin/env python3

import rclpy
import math
import time
import numpy as np
import cv2 as cv

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, Pose

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude

class ControlsPID_Indoor:
    def __init__(self) -> None:
        super().__init__("Controls PID")
        self.get_logger().info("Starting Control System ...")

        global U_yaw
        global Error_yaw

        Kp_Yaw = 0.000
        Ki_Yaw = 0.000
        Kd_Yaw = 0.000

        Ts = 0.250

        global A_Yaw
        global B_Yaw
        global C_Yaw

        A_Yaw = Kp_Yaw + Kd_Yaw / Ts + (Ki_Yaw * Ts) / 2
        B_Yaw = (Ki_Yaw * Ts) / 2 - Kp_Yaw - (2 * Kd_Yaw) / Ts
        C_Yaw = Kd_Yaw / Ts

        U_yaw = [0.0, 0.0]
        Error_yaw = [0.0, 0.0, 0.0]


    def ControlPID_yaw(reference, Error_Actual_Total, High_limit, Low_limit):
        sign = 0

        if Error_Actual_Total - reference >= 0:
            sign = 1
        
        else:
            sign = -1

        Error_yaw[0] = abs(Error_Actual_Total - reference)

        U_yaw[0] = U_yaw[1] + A_Yaw * Error_yaw[0] + B_Yaw * Error_yaw[1] + C_Yaw * Error_yaw[2]    
        
        if  U_yaw[0] > High_limit:
            U_yaw[0] = High_limit
        
        elif U_yaw[0] < Low_limit:
            U_yaw[0] = Low_limit
        
        else:
            U_yaw[0]


        inputControl = sign *  U_yaw[0]
        
        # Publish Topic for velocities

        # ----------------------------------

        U_yaw[1] = U_yaw[0]

        Error_yaw[2] = Error_yaw[1]
        Error_yaw[1] = Error_yaw[0]



