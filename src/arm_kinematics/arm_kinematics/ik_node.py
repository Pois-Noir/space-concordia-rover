import rclpy
from rclpy.node import Node
import drake_ik

class IkService(Node):
    def __init__(self):
        super().__init__('ik_node')
        