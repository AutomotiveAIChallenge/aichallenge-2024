#!/usr/bin/env python3
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.executors
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

class PitStopMarkerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("pitstop_marker")
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/aichallenge/pitstop/area_marker", qos)
        self.callback(None)

    def callback(self, msg):
        markers = MarkerArray()
        markers.markers = [self.create_marker()]
        self.pub.publish(markers)

    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 89626.3671875
        marker.pose.position.y = 43134.921875
        marker.pose.position.z = -29.700000762939453
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = -0.8788172006607056
        marker.pose.orientation.w = -0.47715866565704346
        marker.scale.x = 4.0
        marker.scale.y = 2.0
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ObjectMarkerNode())
    executor.add_node(PitStopMarkerNode())
    executor.spin()
    rclpy.spin(ObjectMarkerNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
