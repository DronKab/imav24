import rclpy
from px4_msgs.msg import VehicleCommand, OffboardControlMode
from rclpy.node import Node

class SendCommand(Node):
    def __init__(self):
        super().__init__("send_command")

        self.command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_MISSION_START, param1=1.0, param2=1.0)

        self.get_logger().info("Command sent")



    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_publisher.publish(msg)


# Definition of main function to create and run the node
def main(args=None):
    rclpy.init(args=args)
    node = SendCommand()
    
    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

# Call main function
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)