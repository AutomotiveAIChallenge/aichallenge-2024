#!/usr/bin/env python3
import rclpy
import rclpy.node
from std_msgs.msg import Bool
from autoware_auto_vehicle_msgs.srv import ControlModeCommand

# Workaround because the simulator cannot use the service.
class ControlModeAdapterNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("control_mode_adapter")
        self.sub = self.create_service(ControlModeCommand, "/control/control_mode_request", self.callback)
        self.pub = self.create_publisher(Bool, "/awsim/control_mode_request_topic", 1)

    def callback(self, req, res):
        msg = Bool()
        if req.mode == ControlModeCommand.Request.AUTONOMOUS:
            res.success = True
            msg.data = True
            self.pub.publish(msg)
            return res
        if req.mode == ControlModeCommand.Request.MANUAL:
            res.success = True
            msg.data = False
            self.pub.publish(msg)
            return res
        res.success = False
        return res

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControlModeAdapterNode())
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
