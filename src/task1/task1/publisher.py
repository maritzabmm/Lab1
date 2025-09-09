import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = '/practice1' # TO DO: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic_name,
            self.qos_profile
            )
        
        self.timer_period = 0.5      # time in seconds
        self.timer = self.create_timer(self.timer_period,
                                        self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # Updating
        self.xrate = 1.0
        self.yrate = 0.0
        self.zrate = 0.0


    def timer_callback(self):
        msg = PoseStamped() 
        msg.pose.position = Point(x=self.x, y=self.y, z=self.z)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Position: {msg.pose.position}')
        
        self.x += self.xrate * self.timer_period
        self.y += self.yrate * self.timer_period
        self.z += self.zrate * self.timer_period
        
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

