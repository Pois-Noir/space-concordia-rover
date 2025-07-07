import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPS(Node):
    def __init__(self):
        super().__init__('added_gps_node')
        self.gps_pub = self.create_publisher(NavSatFix, self.get_name()+'_topic', 10)
        self.create_timer(0.1, self.publish_gps)

    def publish_gps(self):
        msg = NavSatFix()
        msg.latitude = 50.0
        msg.longitude = 60.0
        msg.altitude = 70.0
        msg.position_covariance = [0.0 for i in range(9)]
        msg.position_covariance_type = 3
        self.gps_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPS()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()