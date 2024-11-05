import rclpy
from rclpy.node import Node
from sar_msgs.msg import ROSParams
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class ROS2ParameterSubscriber(Node):
    def __init__(self):
        super().__init__('ROS2_Parameter_Subscriber')
        self.subscriber_ROS2Parameter = self.create_subscription(
            ROSParams,
            '/ROS2/PARAMETER',
            self.ROS2_Parameter_Callback,
            1
        )
        self.tau_diff_data = []

    def ROS2_Parameter_Callback(self, msg):
        # Process the received Optical Flow data here
        self.tau_diff_data.append(msg.tau_diff)
        self.get_logger().info(f"Received Optical Flow data: {msg.tau_diff}")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2ParameterSubscriber()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Graph Setting
    plt.figure()
    plt.xlabel('Time (samples)')
    plt.ylabel('tau_diff')
    plt.title('Real-time tau_diff Plot')

    def update_plot(frame):
        plt.cla()  # Initialize Graph
        plt.plot(node.tau_diff_data, label="tau_diff")
        plt.ylim(-0.1, 0.5)
        plt.xlabel('Time (samples)')
        plt.ylabel('tau_diff')
        plt.title('Real-time tau_diff Plot')
        plt.legend()

    ani = FuncAnimation(plt.gcf(), update_plot, interval=50)
    plt.show()


    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()


if __name__ == '__main__':
    main()