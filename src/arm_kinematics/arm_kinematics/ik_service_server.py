import rclpy
from rclpy.node import Node

class IkService(Node):

    def __init__(self):
        super().__init__('ik_service')
        self.srv = self.create_service(
            srv_type,
            srv_name,
            callback_function)