import rclpy
from rclpy.node import Node
from autoware_planning_msgs.srv import SetRoute
from geometry_msgs.msg import PoseStamped


class SetRouteClientAsync(Node):
    def __init__(self):
        super().__init__("set_route_client_async")
        self.get_logger().info("started")
        self.sub = self.create_subscription(PoseStamped, "/goal_pose", self.callback, 1)
        self.sub  # prevent unused variable warning
        self.cli = self.create_client(SetRoute, "/planning/mission_planning/set_route")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def callback(self, msg):
        self.get_logger().info("I heard: [%s]" % msg)
        self.send_request(msg)

    def send_request(self, goal: PoseStamped):
        request = SetRoute.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "base_link"
        # request.segments
        request.goal = goal.pose

        self.future = self.cli.call_async(request)
        self.future.add_done_callback(self.future_callback)
    
    def future_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
        else:
            self.get_logger().info("Result: %s" % (response))


def main(args=None):
    rclpy.init(args=args)
    node = SetRouteClientAsync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
