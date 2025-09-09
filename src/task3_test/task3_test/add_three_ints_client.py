import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial
from custom_interface.srv import AddThreeInts

class SumClientNode(Node):

    def __init__(self):
        super().__init__('sum_client_node')
        self.get_logger().info('Sum Client Python node has been created')

        # declare parameters 
        a_ = 6
        b_ = 7
        c_ = 8

        self.call_sum_server(a_, b_, c_)

    def call_sum_server(self, a, b, c):
        client = self.create_client(AddThreeInts, 'add_three_ints')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = AddThreeInts.Request()
        request.a = a
        request.b = b
        request.c = c

        #send request asynchronously
        future = client.call_async(request)
        future.add_done_callback(partial(self.sum_service_callback, a=a, b=b, c=c))


    def sum_service_callback(self, future, a, b, c):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {a} + {b} + {c} = {response.sum}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SumClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=='__main__':
    main()