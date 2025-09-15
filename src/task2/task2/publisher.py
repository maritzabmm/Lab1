import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from custom_interface.msg import Person
import random

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = '/practice2' # TO DO: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            Person,
            topic_name,
            self.qos_profile
            )
        
        self.timer_period = 0.5      # time in seconds
        self.timer = self.create_timer(self.timer_period,
                                        self.timer_callback)

        self.i = 1


    def timer_callback(self):
        msg = Person()
        msg.time = Header()
        times = []
        msg.time.stamp = self.get_clock().now().to_msg()
        times.append(msg.time.stamp.sec)
        msg.name = "Susannah"
        msg.age = random.randint(1, 99)
        if msg.age <= 18 and msg.age >= 10:
            msg.is_student = True
        else:
            msg.is_student = False

        self.publisher_.publish(msg)
        self.get_logger().info(f'Time: {msg.time.stamp.sec}.{msg.time.stamp.nanosec} \n'
                               f'Publishing: {msg.name}, age={msg.age}, Student? {msg.is_student}')
        
        self.i += 1
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
