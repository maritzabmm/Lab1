from operator import add
import rclpy 
from rclpy.node import Node 
from custom_interface.action import Countdown 
from rclpy.action import ActionClient 
from rclpy.action.client import ClientGoalHandle 

import time 

class CountdownCancelClientNode(Node): 
    def __init__(self): 
        super().__init__('countdown_cancel_client_node') 
        self.action_client_ = ActionClient( 
            self,  
            Countdown,  
            'countdown_action'
        )

        # get goal parameter value 
        self.declare_parameter('start_from', 5) # default 5 
        self.start_from = self.get_parameter('start_from').value 

        # get time to cancel parameter value
        self.declare_parameter('time_to_cancel', 2.0) # default 2.0 seconds
        self.time_to_cancel = self.get_parameter('time_to_cancel').value 

        self.get_logger().info('Countdown action client has been started')

    def send_goal(self, start_from): 
        """ sends goal request """
        
        self.get_logger().info('Waiting for action server...') 
        self.action_client_.wait_for_server() 
        
        # create a goal 
        goal = Countdown.Goal() 
        goal.start_from = self.start_from 
        
        #Send the goal
        self.get_logger().info('Sending goal request...')
        self.action_client_.send_goal_async(goal, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback): 
        """ logs feedback response """
        self.get_logger().info(f'Received feedback: {feedback.feedback.current}') 

    def goal_response_callback(self, future):
        """ if the goal is accepted, starts a timer. After timeout, timer_callback gets called""" 
        
        goal_handle = future.result() 
        
        if not goal_handle.accepted: 
            self.get_logger().info('Goal rejected :(') 
            return 
        
        self._goal_handle = goal_handle 
        self.get_logger().info('Goal accepted :)')

        #Only cancel the request if time to cancel is greater than 0
        if self.time_to_cancel > 0.0 and self.start_from > self.time_to_cancel:
            # Start a timer for the cancel request
            self._timer = self.create_timer(self.time_to_cancel, self.timer_cancel_goal_callback)

    def timer_cancel_goal_callback(self):
        """ sends cancel goal async request """
        self.get_logger().info('Canceling goal...')

        #Cancel the goal
        future = self._goal_handle.cancel_goal_async().add_done_callback(self.cancel_done)
        
        # Cancel the timer 
        self._timer.cancel()

    def cancel_done(self, future): 
        """ receives cancel goal response and checks if it was successfull""" 
        cancel_response = future.result() 
        
        if len(cancel_response.goals_canceling) > 0: 
            self.get_logger().info('Goal successfully canceled') #Message for the goal successfully canceled
        else: 
            self.get_logger().info('Goal failed to cancel') # Message for the goal failed to cancel
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args) 
    node = CountdownCancelClientNode()

    # send goal 
    node.send_goal(node.start_from)

    rclpy.spin(node)

    try:
        rclpy.shutdown()
    except:
        pass   
    
if __name__ == '__main__':
    main()