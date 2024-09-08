#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera', 10)
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().info("Not able to open camera device")

        ret, frame = self.cap.read()
        self.width = frame.shape[0]
        self.height = frame.shape[1]
        self.get_logger().info(f"Complete resolution is {self.width}x{self.height}")
        self.new_width = int(0.4 * self.width)
        self.new_height = int(0.4 * self.height)
        self.get_logger().info(f"Resizing to {self.new_width}x{self.new_height}")
        self.bridge = CvBridge()

        # Establecer información de la cámara (puedes ajustar estos valores según tu cámara)
        self.camera_info = CameraInfo()
        self.camera_info.width = self.new_width
        self.camera_info.height = self.new_height
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Coeficientes de distorsión
        self.camera_info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Matriz de intrínsecos
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Matriz de rotación
        self.camera_info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # Matriz de proyección
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (self.new_height, self.new_width))
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)

            self.camera_info.header.stamp = self.get_clock().now().to_msg()
            self.camera_info.header.frame_id = 'camera_frame'
            self.camera_info_publisher_.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
