from custom_interface.srv import CalculateDistance
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, Point

class DistanceServerNode(Node):

    def __init__(self):
        super().__init__('distance_server')
        self.srv = self.create_service(CalculateDistance, 'calculate_distance', self.distance_service_callback)
        self.get_logger().info("Service server Python node has been created")

    def distance_service_callback(self, request, response):
        if (math.isfinite(request.p1.x) and math.isfinite(request.p1.y) and math.isfinite(request.p2.x) and math.isfinite(request.p2.y)):
            response.d = math.sqrt((request.p2.x-request.p1.x)**2 + (request.p2.y-request.p1.y)**2)
            response.is_success = True
            response.msg = 'Distance successfully calculated!'
        else:
            response.d = 0.0
            response.is_success = False
            response.msg = 'Invalid values. Try again with float numbers.'

        self.get_logger().info(f'Processed request: Point 1 - ({request.p1.x}, {request.p1.y}), Point 2 - ({request.p2.x}, {request.p2.y}) \n Distance between p1 & p2 = {response.d} \n {response.is_success}, {response.msg}')

        return response

def main(args = None):
 rclpy.init(args=args)
 node = DistanceServerNode()
 rclpy.spin(node)
 rclpy.shutdown()

if __name__=='__main__':
 main()