import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class TFminiPlusNode(Node):
    def __init__(self):
        super().__init__('tfmini_plus_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timer_period', 0.1)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

        self.serial = serial.Serial(port=port, baudrate=baud, timeout=1)
        self.publisher_ = self.create_publisher(Float32, 'range', 10)

        self.get_logger().info(f"TFmini Plus started on {port} @ {baud} baud")

        # self.set_frame_rate(100)
        self.enable_output()
        self.set_frame_rate(100)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def make_command(self, cmd_id: int, payload: list[int]) -> bytearray:
        head = 0x5A
        length = 4 + len(payload)
        frame = [head, length, cmd_id] + payload
        checksum = sum(frame) & 0xFF
        command = bytearray(frame + [checksum])
        # print(f"{hex(b) for b in command}")
        return command

    def enable_output(self):
        cmd = self.make_command(0x07, [0x01])  # enable output: 5A 05 07 01 67
        self.serial.write(cmd)
        self.get_logger().info(f"Sent enable_output: {[hex(b) for b in cmd]}")
        time.sleep(1)

    def disable_output(self):
        cmd = self.make_command(0x07, [0x00])  # disable output: 5A 05 07 00 66
        self.serial.write(cmd)
        self.get_logger().info(f"Sent disable_output: {[hex(b) for b in cmd]}")
        time.sleep(1)
        
    def set_frame_rate(self, hz: int):
        # Frame rate is 2 bytes (little-endian)
        payload = [
            hz & 0xFF,
            (hz >> 8) & 0xFF
        ]
        cmd = self.make_command(0x03, payload)
        self.serial.write(cmd)
        self.get_logger().info(f"Sent frame rate set command: {[hex(b) for b in cmd]}")
        time.sleep(1)

    def timer_callback(self):
        while self.serial.in_waiting >= 9:
            b1 = self.serial.read(1)
            if b1[0] != 0x59:
                continue
            b2 = self.serial.read(1)
            if b2[0] != 0x59:
                continue
            frame = b1 + b2 + self.serial.read(7)
            if len(frame) != 9:
                self.get_logger().warn("Incomplete frame")
                return

            checksum = sum(frame[0:8]) & 0xFF
            if checksum != frame[8]:
                self.get_logger().warn("Checksum mismatch")
                return

            distance_mm = frame[2] + (frame[3] << 8)
            msg = Float32()
            msg.data = distance_mm / 1000.0
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published: {msg.data:.3f} m")
            # self.get_logger().info(f"Published: {msg.data:.3f} m")
            break

    def destroy_node(self):
        self.disable_output()
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

