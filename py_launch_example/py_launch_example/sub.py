import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubNode(Node):
    def __init__(self):
        super().__init__('sub_node')
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
