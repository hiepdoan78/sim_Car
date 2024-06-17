import rclpy
from rclpy.node import Node
import math
import serial
from threading import Lock

class ServoDriver(Node):

    def __init__(self):
        super().__init__('servo_driver')
        
        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value
        
        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value
        
        self.declare_parameter('max_angle', value=180)  # Giá trị góc tối đa
        self.max_angle = self.get_parameter('max_angle').value

        self.mutex = Lock()

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")

    def send_servo_command(self, angle):
        if 0 <= angle <= self.max_angle:
            cmd_string = f"s {int(angle)}\r"
            self.mutex.acquire()
            try:
                self.conn.write(cmd_string.encode("utf-8"))
                print(f"Sent: {cmd_string}")
            finally:
                self.mutex.release()
        else:
            print("Error: Angle out of range")

    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)
    servo_driver = ServoDriver()

    try:
        while rclpy.ok():
            angle = float(input("Enter angle (0-180): "))
            servo_driver.send_servo_command(angle)
    except KeyboardInterrupt:
        pass
    finally:
        servo_driver.close_conn()
        servo_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
