import rclpy
from rclpy.node import Node
from sar_msgs.msg import SARStateData

class StateDataSubscriber(Node):

    def __init__(self):
        super().__init__('state_data_subscriber')
        self.subscription = self.create_subscription(
            SARStateData,
            '/SAR_DC/StateData',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print('Received state data:')
        self.get_logger().info(f'Received state data: {msg}')

def main(args=None):
    rclpy.init(args=args)
    state_data_subscriber = StateDataSubscriber()
    rclpy.spin(state_data_subscriber)
    state_data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()