import rclpy
from rclpy.node import Node
from tkinter import *
import math

from serial_motor_demo_msgs.msg import MotorCommand


class MotorGui(Node):

    def __init__(self):
        super().__init__('motor_gui')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)

        self.tk = Tk()
        self.tk.title("Serial Motor GUI")
        root = Frame(self.tk)
        root.pack(fill=BOTH, expand=True)

        Label(root, text="Serial Motor GUI").pack()

        servo_frame = Frame(root)
        servo_frame.pack(fill=X)
        Label(servo_frame, text="Servo Angle").pack(side=LEFT)
        self.servo = Scale(servo_frame, from_=-90, to=90, orient=HORIZONTAL)
        self.servo.pack(side=LEFT, fill=X, expand=True)

        motor_btns_frame = Frame(root)
        motor_btns_frame.pack()
        Button(motor_btns_frame, text='Send Once', command=self.send_servo_once).pack(side=LEFT)
        Button(motor_btns_frame, text='Stop Servo', command=self.stop_servo).pack(side=LEFT)

        self.set_mode(True)

    def send_servo_once(self):
        msg = MotorCommand()
        msg.is_pwm = True  # Assume PWM mode for simplicity
        msg.mot_1_req_rad_sec = float(self.servo.get()) * (math.pi / 180)  # Convert to radians
        msg.mot_2_req_rad_sec = 0.0  # Assume only one motor for steering
        self.publisher.publish(msg)

    def stop_servo(self):
        msg = MotorCommand()
        msg.is_pwm = True  # Assume PWM mode for simplicity
        msg.mot_1_req_rad_sec = 0.0
        msg.mot_2_req_rad_sec = 0.0
        self.publisher.publish(msg)

    def update(self):
        self.tk.update()


def main(args=None):
    rclpy.init(args=args)
    motor_gui = MotorGui()
    rate = motor_gui.create_rate(20)
    while rclpy.ok():
        rclpy.spin_once(motor_gui)
        motor_gui.update()

    motor_gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
