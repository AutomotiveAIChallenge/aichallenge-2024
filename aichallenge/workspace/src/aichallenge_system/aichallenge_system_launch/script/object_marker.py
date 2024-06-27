#!/usr/bin/env python3
import rclpy
import rclpy.node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


class ObjectMarkerNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("object_marker")
        self.sub = self.create_subscription(Float64MultiArray, "/aichallenge/objects", self.callback, 1)
        self.pub = self.create_publisher(MarkerArray, "/aichallenge/objects_marker", 1)

    def callback(self, msg):
        markers = MarkerArray()
        markers.markers = [self.create_marker(msg.data, i) for i in range(0, len(msg.data), 4)]
        self.pub.publish(markers)

    def create_marker(self, data, i):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = i // 4
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = data[i + 0]
        marker.pose.position.y = data[i + 1]
        marker.pose.position.z = data[i + 2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = data[i + 3] * 2
        marker.scale.y = data[i + 3] * 2
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObjectMarkerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
