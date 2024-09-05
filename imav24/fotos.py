import rclpy #Crea y maneja nodos.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleGlobalPosition
from rclpy.node import Node

class fotos(Node):
    def __init__(self):
        super().__init__("send_command")
        self.get_logger().info("Go to Waypoint started")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.drone_attitude_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.vehicle_global_callback, qos_profile)
        self.timers = self.create_timer(0.1, self.controlPosition)


# Definition of main function to create and run the node
def main(args=None):
    rclpy.init(args=args)
    node = fotos()
    
    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

# Call main function
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)