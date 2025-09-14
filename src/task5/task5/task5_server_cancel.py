import rclpy 
from rclpy.node import Node 
from custom_interface.action import Countdown 
from rclpy.action import ActionServer 
from rclpy.action import CancelResponse 
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor 
import time 

class CountdownServerNode(Node):
    def __init__(self):
        super().__init__('countdown_server_node')
        
        self.action_server_ = ActionServer(
            self,
            Countdown,
            'countdown_action',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback)
        
        self.get_logger().info('Countdown Action Server has been started.')

    def cancel_callback(self, cancel_request): 
        """accepts or rejects a client request to cancel""" 

        self.get_logger().info('received cancel request') 
        return CancelResponse.ACCEPT 

    def execute_callback(self, goal_handle):
        """execute the action and return the result""" 
        self.get_logger().info(f'Executing the goal {goal_handle.goal_id.uuid}')

        # Get goal request value
        start_from = goal_handle.request.start_from

        #log to review start from
        self.get_logger().info(f'start_from: {start_from}')

        # Initialize feedback
        feedback_msg = Countdown.Feedback()
        feedback_msg.current = start_from

        # Process countdown
        for i in range(start_from, 0, -1):
            
            self.get_logger().info(str(goal_handle.is_cancel_requested))
            
            if goal_handle.is_cancel_requested: 
                goal_handle.canceled() 
                self.get_logger().info('Action canceled') 

                return Countdown.Result()

            feedback_msg.current = i

            self.get_logger().info(f'Feedback: {feedback_msg.current}')
            
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)  
        
        # Set goal state as succeeded
        goal_handle.succeed()

        # Return the result
        result = Countdown.Result()
        result.result_text = "Countdown completed!"
               
        return result
    
def main(args=None):
    rclpy.init(args=args)
    node = CountdownServerNode()

    executor = MultiThreadedExecutor() 
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print('KeyBoardInterrupt')

if __name__ == '__main__':
    main()