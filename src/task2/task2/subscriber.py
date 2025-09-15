import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from custom_interface.msg import Person
import random

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        topic_name = '/practice2' 
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.subscription = self.create_subscription(
            Person,
            topic_name,
            self.listener_callback,
            self.qos_profile)
        
        self.subscription

        self.last_time = None


    def listener_callback(self, msg):
        time_diff = 0.0
        if self.last_time is not None:
            curr = msg.time.stamp
            time_diff = (curr.sec + curr.nanosec * 1e-9) - (self.last_time.sec + self.last_time.nanosec * 1e-9)

        self.last_time = msg.time.stamp

        self.get_logger().info(f'Received at {msg.time.stamp.sec}.{msg.time.stamp.nanosec}: \n' 
                               f'Time difference between actual and last message: {time_diff} \n'
                               f'{msg.name} is {msg.age} years old. Student? {msg.is_student}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
