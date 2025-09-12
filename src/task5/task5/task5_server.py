import rclpy 
from rclpy.node import Node 
from custom_interface.action import Countdown 
from rclpy.action import ActionServer 
import time 

class Task5ServerNode(Node):
    def __init__(self):
        super().__init__('countdown_server_node')
        self.action_server_ = ActionServer(
            self,
            Countdown,
            'countdown_action',
            execute_callback=self.execute_callback)
        self.get_logger().info('Countdown Action Server has been started.')

    def execute_callback(self, goal_handle):
        """execute the action and return the result""" 
        self.get_logger().info('Executing the goal')

        # Get goal request value
        start_from = goal_handle.request.start_from

        # Initialize feedback
        feedback_msg = Countdown.Feedback()
        feedback_msg.current = start_from

        # Process countdown
        for i in range(start_from, 0, -1):
            self.get_logger().info(f'Countdown at: {i}')
            feedback_msg.current = i
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # Simulate time-consuming task (delay in seconds)

        # Set goal state as succeeded
        goal_handle.succeed()

        # Return the result
        result = Countdown.Result()
        result.result_text = "Countdown completed!"

        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = Task5ServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()