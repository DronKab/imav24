import rclpy #Crea y maneja nodos.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleGlobalPosition
from rclpy.node import Node

class GoToWaypoint(Node):
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

        # Variables para almacenar la posición actual
        self.latiActual = 0.0
        self.altiActual = 0.0
        self.longiActual = 0.0  

        # Variables del controlador PID para altitud
        self.altitude_setpoint = 0.0  # Setpoint de altitud en metros
        self.kp_altitude = 0.0          # Ganancia proporcional
        self.ki_altitude = 0.0          # Ganancia integral
        self.kd_altitude = 0.0         # Ganancia derivativa

        self.altitude_integral = 0.0
        self.altitude_error_prev = 0.0

        # Variables del controlador PID para latitud
        self.latitude_setpoint = 0.0  # Ejemplo: Setpoint de latitud en grados
        self.kp_latitude = 0.0            # Ganancia proporcional
        self.ki_latitude = 0.0            # Ganancia integral
        self.kd_latitude = 0.0           # Ganancia derivativa

        self.latitude_integral = 0.0
        self.latitude_error_prev = 0.0

        # Variables del controlador PID para longitud
        self.longitude_setpoint = 0.0  # Ejemplo: Setpoint de longitud en grados
        self.kp_longitude = 0.0             # Ganancia proporcional
        self.ki_longitude = 0.0             # Ganancia integral
        self.kd_longitude = 0.0            # Ganancia derivativa

        self.longitude_integral = 0.0
        self.longitude_error_prev = 0.0

    def vehicle_global_position(self,msg):
        self.latiActual = msg.lat
        self.altiActual = msg.alt
        self.longiActual = msg.lon

    def controlPosition(self):
        ### Control de Altitud
        altitude_error = self.altitude_setpoint - self.altiActual
        p_term_altitude = self.kp_altitude * altitude_error
        self.altitude_integral += altitude_error * 0.1
        i_term_altitude = self.ki_altitude * self.altitude_integral
        d_term_altitude = self.kd_altitude * (altitude_error - self.altitude_error_prev) / 0.1
        pid_output_altitude = p_term_altitude + i_term_altitude + d_term_altitude
        self.altitude_error_prev = altitude_error
        self.get_logger().info(f"Control PID Altitud: {pid_output_altitude:.2f}")

        ### Control de Latitud
        latitude_error = self.latitude_setpoint - self.latiActual
        p_term_latitude = self.kp_latitude * latitude_error
        self.latitude_integral += latitude_error * 0.1
        i_term_latitude = self.ki_latitude * self.latitude_integral
        d_term_latitude = self.kd_latitude * (latitude_error - self.latitude_error_prev) / 0.1
        pid_output_latitude = p_term_latitude + i_term_latitude + d_term_latitude
        self.latitude_error_prev = latitude_error
        self.get_logger().info(f"Control PID Latitud: {pid_output_latitude:.6f}")

        ### Control de Longitud
        longitude_error = self.longitude_setpoint - self.longiActual
        p_term_longitude = self.kp_longitude * longitude_error
        self.longitude_integral += longitude_error * 0.1
        i_term_longitude = self.ki_longitude * self.longitude_integral
        d_term_longitude = self.kd_longitude * (longitude_error - self.longitude_error_prev) / 0.1
        pid_output_longitude = p_term_longitude + i_term_longitude + d_term_longitude
        self.longitude_error_prev = longitude_error
        self.get_logger().info(f"Control PID Longitud: {pid_output_longitude:.6f}")

        ### Publicar los comandos calculados









# Definition of main function to create and run the node
def main(args=None):
    rclpy.init(args=args)
    node = GoToWaypoint()
    
    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

# Call main function
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)