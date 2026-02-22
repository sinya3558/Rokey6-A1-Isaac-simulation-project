import rclpy
from rclpy.node import Node

class MyTestNode(Node):
    def __init__(self):
        super().__init__('my_test_node')
        self.get_logger().info('Hello, ROS2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyTestNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()