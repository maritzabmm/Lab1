import rclpy 
from rclpy.node import Node 
from custom_interface.action import Countdown 
from rclpy.action import ActionClient 
from rclpy.action.client import ClientGoalHandle 
import time 

class Task5ClientNode(Node): 
    def __init__(self): 
        super().__init__('countdown_client_node') 
        self.action_client_ = ActionClient( 
            self,  
            Countdown,  
            'countdown_action'
        )

        # get goal parameter value 
        self.start_from = 5 # TODO fill 

        self.declare_parameter('start_from', 5) # default 5 
        self.start_from = self.get_parameter('start_from').value 

        self.get_logger().info('Countdown action client has been started')

    def send_goal(self, start_from): 
        """the action client sends the goal""" 

        # wait for the server 
        timeout_ = 1.0 
        self.action_client_.wait_for_server(timeout_sec=timeout_)

        # create a goal 
        goal = Countdown.Goal() 
        goal.start_from = start_from

        # send the goal 
        self.get_logger().info(f'sending the goal: {start_from}') 
        self.action_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg): 
        """gets feedback message from feedback_callback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current}') 

    def goal_response_callback(self, future):
        """checks if the goal request was accepted""" 
        self.goal_handle_ : ClientGoalHandle = future.result() 
        # checks if the goal has been accepted 
        if not self.goal_handle_.accepted: 
            self.get_logger().info('Goal rejected ') 
            return
        
        # if the goal has been accepted we can request for the result 
        self.get_logger().info('Goal accepted')
        self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """gets the result """
        result = future.result().result 
        countdown_result = result.result_text

        self.get_logger().info(f'{countdown_result}')

def main(args=None):
    rclpy.init(args=args) 
    node = Task5ClientNode()

    # send goal 
    node.send_goal(node.start_from)

    rclpy.spin(node)

    try:
        rclpy.shutdown()
    except:
        pass   
    
if __name__ == '__main__':
    main()