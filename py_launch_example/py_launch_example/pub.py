import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'hello_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello world!'
        self.get_logger().info(f'Publish: {msg.data}')
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
