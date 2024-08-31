import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class JoystickControl(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.publisher_ = self.create_publisher(Twist, '/px4_driver/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.check_joystick_state)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def check_joystick_state(self):
        pygame.event.pump()
        twist = Twist()

        # Obtener los valores de los ejes
        axis_pitch = self.joystick.get_axis(4) 
        axis_roll = self.joystick.get_axis(3) 

        axis_up_down = self.joystick.get_axis(1)
        axis_yaw = self.joystick.get_axis(0)

        # Mapear los ejes al mensaje Twist
        twist.linear.x = -axis_pitch
        twist.linear.y = -axis_roll 
        twist.linear.z = -axis_up_down 
        twist.angular.z = axis_yaw 

        twist.linear.x = float(round(twist.linear.x))
        twist.linear.y = float(round(twist.linear.y))
        twist.linear.z = float(round(twist.linear.z))
        twist.angular.z = float(round(twist.angular.z))

        # Publicar el mensaje
        if abs(twist.linear.x) + abs(twist.linear.y) + abs(twist.linear.z) + abs(twist.angular.z) > 0:
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)

    joystick_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()