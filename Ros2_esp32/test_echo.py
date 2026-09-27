import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            '/rudder_pos',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(f'Recebi do fg_bridge: "{msg.data}"')
        import sys
        sys.exit(0)

rclpy.init()
minimal_subscriber = MinimalSubscriber()
rclpy.spin(minimal_subscriber)
