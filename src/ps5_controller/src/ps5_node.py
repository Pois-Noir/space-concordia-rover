import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import pygame

pygame.init()
pygame.joystick.init()

class Ps5Controller(Node):
    def __init__(self):
        super().__init__('ps5_node')
        self.rover_pub = self.create_publisher(Twist, '/cmd_wheel', 10)
        self.arm_pub = self.create_publisher(JointState, '/cmd_joint', 10)
        self.ps5 = pygame.joystick.Joystick(0)
        self.ps5.init()

    def controller_callback(self):
        button_layout = {
            'left_x': ps5.get_axis(0),
            'left_y': -ps5.get_axis(1),
            'right_x': ps5.get_axis(3),
            'right_y': -ps5.get_axis(4),
            'left_button': ps5.get_button(11),
            'right_button': ps5.get_button(12),

            'left_trigger': ps5.get_axis(2),
            'right_trigger': ps5.get_axis(5),
            'left_trigger_button': ps5.get_button(6),
            'right_trigger_button': ps5.get_button(7),

            'button_cross': ps5.get_button(0),
            'button_circle': ps5.get_button(1),
            'button_triangle': ps5.get_button(2),
            'button_square': ps5.get_button(3),

            'left_bumper': ps5.get_button(4),
            'right_bumper': ps5.get_button(5),

            'd_pad': ps5.get_hat(0),

            'share_button': ps5.get_button(8),
            'options_button': ps5.get_button(9),
            'ps_button': ps5.get_button(10)
        }
        twist = Twist()
        twist.linear.x = float(button_layout['left_y'])
        twist.angular.y = float(button_layout['right_x'])

        joint_state = JointState()
        joint_state.name = [f'joint{i+1}' for i in range(2)]
        joint_state.velocity = [
            float(button_layout['left_x']),
            float(button_layout['right_y'])
        ]
        joint_state.position = []
        joint_state.effort = []

        self.arm_pub.publish(joint_state)
        self.rover_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('ps5_controller')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()