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
        if hz < 1 or hz > 1000 or (1000 % hz != 0):
            self.get_logger().warn("Invalid frame rate. Must divide 1000 exactly and be in 1~1000 Hz.")
            return
        payload = [hz & 0xFF, (hz >> 8) & 0xFF]
        cmd = self.make_command(0x03, payload)
        self.serial.write(cmd)
        self.get_logger().info(f"Sent frame rate set command: {[hex(b) for b in cmd]}")
        time.sleep(1)

    def timer_callback(self):
        while self.serial.in_waiting >= 9:
            header = self.serial.read(2)
            if header[0] != 0x59 or header[1] != 0x59:
                continue
            frame = header + self.serial.read(7)
            if len(frame) != 9:
                self.get_logger().warn("Incomplete frame")
                return
            checksum = sum(frame[0:8]) & 0xFF
            if checksum != frame[8]:
                self.get_logger().warn("Checksum mismatch")
                return
            distance_cm = frame[2] + (frame[3] << 8)
            strength = frame[4] + (frame[5] << 8)
            if strength < 100 or strength == 65535:
                self.get_logger().warn("Weak signal, ignoring frame")
                return
            msg = Float32()
            msg.data = distance_cm / 100.0
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published: {msg.data:.2f} m")
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
