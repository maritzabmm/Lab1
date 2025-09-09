import sys
import rclpy
from rclpy.node import Node
from functools import partial
from custom_interface.srv import CalculateDistance
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class DistanceClientNode(Node):

    def __init__(self):
        super().__init__('distance_client_node')
        self.get_logger().info('Distance Client Python node has been created')

        # dparameters
        p1 = Point(x=0.0, y=4.0, z=0.0)
        p2 = Point(x=5.0, y=2.0, z=0.0)

        #suc = True
        #msg = 'Successfully calculated!'

        self.call_distance_server(p1, p2)

    def call_distance_server(self, p1, p2):
        client = self.create_client(CalculateDistance, 'calculate_distance')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = CalculateDistance.Request()
        request.p1 = p1
        request.p2 = p2

        # send request 
        future = client.call_async(request)
        future.add_done_callback(partial(self.distance_service_callback, p1=p1, p2=p2))


    def distance_service_callback(self, future, p1, p2):
        try:
            response = future.result()
            self.get_logger().info(f'Result: Distance between ({p1.x},{p1.y}) and ({p2.x},{p2.y}) is {response.d} \n {response.is_success}, {response.msg}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=='__main__':
    main()