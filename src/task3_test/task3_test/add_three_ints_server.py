from custom_interface.srv import AddThreeInts    
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class SumServerNode(Node):

    def __init__(self):
        super().__init__('sum_server')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.sum_service_callback)
        self.get_logger().info("Service server Python node has been created")

    def sum_service_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  
        self.get_logger().info(f'Processed request: {request.a} + {request.b} + {request.c} -> {response.sum}')

        return response

def main(args = None):
 rclpy.init(args=args)
 node = SumServerNode()
 rclpy.spin(node)
 rclpy.shutdown()

if __name__=='__main__':
 main()