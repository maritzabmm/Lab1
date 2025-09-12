import sys
import rclpy
from rclpy.node import Node
from functools import partial
from custom_interface.srv import CalculateDistance
from geometry_msgs.msg import Point

class DistanceClientNode(Node):

    def __init__(self):
        super().__init__('distance_client_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('x1', 1.0),
                ('y1', 1.0),
                ('x2', 1.0),
                ('y2', 1.0),
            ])
        
        x1 = self.get_parameter('x1').value
        y1 = self.get_parameter('y1').value
        x2 = self.get_parameter('x2').value
        y2 = self.get_parameter('y2').value

        p1 = Point(x=x1, y=y1, z=0.0)
        p2 = Point(x=x2, y=y2, z=0.0)
        
        self.get_logger().info('Distance Client Python node has been created')
        self.call_distance_server(p1, p2)

    def call_distance_server(self, p1, p2):
        client = self.create_client(CalculateDistance, 'calculate_distance')
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        request = CalculateDistance.Request()
        request.p1 = p1
        request.p2 = p2

        future = client.call_async(request)
        future.add_done_callback(partial(self.distance_service_callback, p1=p1, p2=p2))

    def distance_service_callback(self, future, p1, p2):
        try:
            response = future.result()
            self.get_logger().info(
                f'Result: Distance between ({p1.x},{p1.y}) and ({p2.x},{p2.y}) is {response.d} \n Success: {response.is_success}, {response.msg}'
            )
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceClientNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
