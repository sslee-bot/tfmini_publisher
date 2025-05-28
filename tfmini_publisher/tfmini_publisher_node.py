import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TFminiPlusNode(Node):
    def __init__(self):
        super().__init__('tfmini_plus_node')

        port = '/dev/ttyS0'
        baud = 115200
        self.serial = serial.Serial(port=port, baudrate=baud, timeout=1)
        self.publisher_ = self.create_publisher(Float32, 'range', 10)

        self.get_logger().info(f"TFmini Plus started on {port} @ {baud} baud")

        self.buffer = bytearray()
        self.latest_distance = None

        # Declare and get the desired publish and read intervals
        self.declare_parameter('publish_period', 0.1)
        self.declare_parameter('reader_period', 0.01)
        publish_period = self.get_parameter('publish_period').get_parameter_value().double_value
        reader_period = self.get_parameter('reader_period').get_parameter_value().double_value

        # Read raw data at reader_period
        self.reader_timer = self.create_timer(reader_period, self.reader_callback)
        # Publish data at the user-defined interval
        self.publisher_timer = self.create_timer(publish_period, self.publisher_callback)

    def reader_callback(self):
        self.buffer += self.serial.read(self.serial.in_waiting or 1)

        while len(self.buffer) >= 9:
            if self.buffer[0] != 0x59 or self.buffer[1] != 0x59:
                self.buffer.pop(0)
                continue

            frame = self.buffer[:9]
            checksum = sum(frame[0:8]) & 0xFF
            if checksum != frame[8]:
                self.get_logger().warn("Checksum mismatch")
                self.buffer.pop(0)
                continue

            distance_cm = frame[2] + (frame[3] << 8)
            strength = frame[4] + (frame[5] << 8)

            if strength < 100 or strength == 65535:
                self.get_logger().warn("Weak signal, ignoring frame")
                self.buffer = self.buffer[9:]
                continue

            self.latest_distance = distance_cm / 100.0
            self.buffer = self.buffer[9:]
            continue

    def publisher_callback(self):
        if self.latest_distance is not None:
            msg = Float32()
            msg.data = self.latest_distance
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published: {msg.data:.2f} m")

    def destroy_node(self):
        self.serial.close()
        self.get_logger().info("Sensor shut down.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TFminiPlusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

