import rclpy
from rclpy.node import Node
import serial
# Modify import based on actual message type
# Using 64 bit float array for now
# from std_msgs.msg import Float64MultiArray
from stewart_interfaces import Velocity

class JetsonSerial(Node):
    def __init__(self):
            super().__init__('jetson_serial')
            
            # Subscriber Initialization
            self.subscription = self.create_subscription(
                  Velocity,
                  'motor-velocity',
                  self.listener_callback,
                  10)
            self.subscription
            
            # Serial Initialization
            # Need to check the port and the baudrate
            self.ser = serial.Serial(port = '/dev/ttyUSB0',
                                     baudrate = 115200,
                                     timeout = 1)

    def listener_callback(self, msg):
        data = msg.data

        # Data length check in case it is imcomplete
        if len(data) < 7:
            self.get_logger().warn("Received Incomplete Message")
            return
        
        # Format of string
        str_format = "<" + ",".join([f"{x:.4f}" for x in data]) + ">"

        # Writing String to Arduino
        self.ser.write((str_format + "\n").encode('utf-8'))
        self.get_logger().info(f"Sent: {str_format}")

def main(args=None):
    rclpy.init(args=args)

    node = JetsonSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            
