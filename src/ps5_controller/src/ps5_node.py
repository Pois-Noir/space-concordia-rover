import rclpy
import rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import pygame

left_x = joystick.get_axis(0)
left_y = -joystick.get_axis(1)
right_x = joystick.get_axis(3)
right_y = -joystick.get_axis(4)
left_button = joystick.get_button(11)
right_button = joystick.get_button(12)

left_trigger = joystick.get_axis(2)
right_trigger = joystick.get_axis(5)
left_trigger_button = joystick.get_button(6)
right_trigger_button = joystick.get_button(7)

button_cross = joystick.get_button(0)
button_circle = joystick.get_button(1)
button_triangle = joystick.get_button(2)
button_square = joystick.get_button(3)

left_bumper = joystick.get_button(4)
right_bumper = joystick.get_button(5)

d_pad = joystick.get_hat(0)

share_button = joystick.get_button(8)
options_button = joystick.get_button(9)
ps_button = joystick.get_button(10)

class ps5_controller(Node):
    def __init__(self):
        super().__init__('ps5_node')
        self.rover_pub = self.create_publisher(Twist, '/cmd_wheel', 10)
        self.arm_pub = self.create_publisher(JointState, '/cmd_joint', 10)
