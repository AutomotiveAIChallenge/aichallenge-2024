import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_auto_control_msgs.msg import AckermannControlCommand
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile
import math
import torch
import numpy as np
import tf_transformations
from mppi_controller.mppi_controller import mppi_controller
from mppi_controller.cost_map_tensor import CostMapTensor

class MppiControllerNode(Node):

    def __init__(self):
        super().__init__('mppi_controller_node')

        # parameters
        # declare
        self.declare_parameter('horizon', 25)
        self.declare_parameter('num_samples', 4000)
        self.declare_parameter('u_min', [-2.0, -0.25])
        self.declare_parameter('u_max', [2.0, 0.25])
        self.declare_parameter('sigmas', [0.5, 0.1])
        self.declare_parameter('lambda', 1.0)
        self.declare_parameter('auto_lambda', False)
        self.declare_parameter('DL', 0.1)
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('reference_path_interval', 0.85)
        self.declare_parameter('Qc', 2.0)
        self.declare_parameter('Ql', 3.0)
        self.declare_parameter('Qv', 2.0)
        self.declare_parameter('Qo', 10000.0)
        self.declare_parameter('Qin', 0.01)
        self.declare_parameter('Qdin', 0.5)
        self.declare_parameter('delta_t', 0.1)
        self.declare_parameter('vehicle_L', 1.0)
        self.declare_parameter('V_MAX', 8.0)
        # get
        self.config = {
            "horizon": self.get_parameter('horizon').get_parameter_value().integer_value,
            "num_samples": self.get_parameter('num_samples').get_parameter_value().integer_value,
            "u_min": self.get_parameter('u_min').get_parameter_value().double_array_value,
            "u_max": self.get_parameter('u_max').get_parameter_value().double_array_value,
            "sigmas": self.get_parameter('sigmas').get_parameter_value().double_array_value,
            "lambda": self.get_parameter('lambda').get_parameter_value().double_value,
            "auto_lambda": self.get_parameter('auto_lambda').get_parameter_value().bool_value,
            "DL": self.get_parameter('DL').get_parameter_value().double_value,
            "lookahead_distance": self.get_parameter('lookahead_distance').get_parameter_value().double_value,
            "reference_path_interval": self.get_parameter('reference_path_interval').get_parameter_value().double_value,
            "Qc": self.get_parameter('Qc').get_parameter_value().double_value,
            "Ql": self.get_parameter('Ql').get_parameter_value().double_value,
            "Qv": self.get_parameter('Qv').get_parameter_value().double_value,
            "Qo": self.get_parameter('Qo').get_parameter_value().double_value,
            "Qin": self.get_parameter('Qin').get_parameter_value().double_value,
            "Qdin": self.get_parameter('Qdin').get_parameter_value().double_value,
            "delta_t": self.get_parameter('delta_t').get_parameter_value().double_value,
            "vehicle_L": self.get_parameter('vehicle_L').get_parameter_value().double_value,
            "V_MAX": self.get_parameter('V_MAX').get_parameter_value().double_value,
        }
        self.get_logger().info(f'config: {self.config}')

        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # publisher
        # control command
        self.pub_cmd = self.create_publisher(AckermannControlCommand, 'output/control_cmd', 1)
        # planned path
        self.pub_planned_path = self.create_publisher(Trajectory, 'output/planned_path', 1)
        # debug path
        self.pub_debug_path = self.create_publisher(Trajectory, 'debug/path', 1)

        # subscriber
        # state
        self.sub_kinematics = self.create_subscription(
            Odometry,
            'input/kinematics',
            self.kinematics_callback,
            1
        )
        # reference trajectory
        self.sub_trajectory = self.create_subscription(
            Trajectory,
            'input/reference_trajectory',
            self.trajectory_callback,
            1
        )
        # costmap
        self.sub_costmap = self.create_subscription(
            OccupancyGrid,
            'input/costmap',
            self.costmap_callback,
            1
        )

        # device and dtype
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.dtype = torch.float32
        
        # mppi controller
        self.controller = mppi_controller(config=self.config, debug=True, device=self.device, dtype=self.dtype)

        self.odometry: Odometry = None
        self.trajectory: Trajectory = None
        self.costmap: OccupancyGrid = None

        self.timer = self.create_timer(0.03, self.on_timer)

    def kinematics_callback(self, msg : Odometry):
        self.odometry = msg

    def trajectory_callback(self, msg : Trajectory):
        self.trajectory = msg

    def costmap_callback(self, msg : OccupancyGrid):
        self.costmap = msg

    def parameter_callback(self, params):
        for param in params:
            if param.name in self.config:
                if param.type_ == Parameter.Type.DOUBLE:
                    self.config[param.name] = param.value
                elif param.type_ == Parameter.Type.INTEGER:
                    self.config[param.name] = param.value
                elif param.type_ == Parameter.Type.BOOL:
                    self.config[param.name] = param.value
                elif param.type_ == Parameter.Type.DOUBLE_ARRAY:
                    self.config[param.name] = param.value
                self.get_logger().info(f"Parameter {param.name} changed to {param.value}")

        # update controller
        self.controller.update_params(self.config)

        # Return a success result
        return SetParametersResult(successful=True)

    def zero_ackermann_control_command(self):
        cmd = AckermannControlCommand()
        now = self.get_clock().now().to_msg()
        cmd.stamp = now
        cmd.longitudinal.stamp = now
        cmd.longitudinal.speed = 0.0
        cmd.longitudinal.acceleration = 0.0
        cmd.lateral.stamp = now
        cmd.lateral.steering_tire_angle = 0.0
        return cmd

    def on_timer(self):
        if not self.subscribe_message_available():
            return

        cmd = self.zero_ackermann_control_command()

        # convert tensor
        # state
        state_tensor = torch.tensor([
            self.odometry.pose.pose.position.x,
            self.odometry.pose.pose.position.y,
            tf_transformations.euler_from_quaternion([
                self.odometry.pose.pose.orientation.x,
                self.odometry.pose.pose.orientation.y,
                self.odometry.pose.pose.orientation.z,
                self.odometry.pose.pose.orientation.w
            ])[2],
            self.odometry.twist.twist.linear.x
        ], dtype=self.dtype, device=self.device)
        # reference path
        reference_path_tensor = torch.tensor([
            [point.pose.position.x, 
             point.pose.position.y, 
             tf_transformations.euler_from_quaternion([
                point.pose.orientation.x,
                point.pose.orientation.y,
                point.pose.orientation.z,
                point.pose.orientation.w
             ])[2], 
             point.longitudinal_velocity_mps] for point in self.trajectory.points
        ], dtype=self.dtype, device=self.device)
        # convert OccupancyGrid to tensor
        costmap_tensor: CostMapTensor = CostMapTensor(
            cost_map=torch.tensor(self.costmap.data, dtype=self.dtype, device=self.device).reshape(self.costmap.info.height, self.costmap.info.width),
            cell_size=self.costmap.info.resolution,
            origin=(self.costmap.info.origin.position.x, 
                    self.costmap.info.origin.position.y),
            device=self.device,
            dtype=self.dtype
        )
        # set cost map
        self.controller.set_cost_map(costmap_tensor)

        # update controller
        action_seq, state_seq = self.controller.update(state_tensor, reference_path_tensor)
        # get top samples
        top_samples = self.controller.get_top_samples(num_samples=300)

        # convert numpy
        state = state_seq.cpu().numpy()[0]
        action = action_seq.cpu().numpy()[0]
        
        # publish control command
        cmd.longitudinal.speed = float(state[0, 3])
        cmd.longitudinal.acceleration = float(action[0])
        cmd.lateral.steering_tire_angle = float(action[1])
        self.pub_cmd.publish(cmd)

        # publish planned path
        planned_path = Trajectory()
        planned_path.header.stamp = self.get_clock().now().to_msg()
        planned_path.header.frame_id = 'map'
        planned_path.points = [
            TrajectoryPoint(
                pose=Pose(
                    position=Point(x=float(point[0]), y=float(point[1])),
                    orientation=Quaternion(
                        x=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[0],
                        y=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[1],
                        z=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[2],
                        w=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[3]
                    )
                ),
                longitudinal_velocity_mps=float(point[3])
            ) 
        for point in state]

        self.pub_planned_path.publish(planned_path)

        # publish debug path
        debug_path = Trajectory()
        debug_path.header.stamp = self.get_clock().now().to_msg()
        debug_path.header.frame_id = 'map'
        debug_path.points = [
            TrajectoryPoint(
                pose=Pose(
                    position=Point(x=float(point[0]), y=float(point[1])),
                    orientation=Quaternion(
                        x=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[0],
                        y=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[1],
                        z=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[2],
                        w=tf_transformations.quaternion_from_euler(0.0, 0.0, float(point[2]))[3]
                    )
                ),
                longitudinal_velocity_mps=float(point[3])
            ) 
        for point in self.controller.reference_path.cpu().numpy()]
        self.pub_debug_path.publish(debug_path)


    def subscribe_message_available(self):
        if not self.odometry:
            self.get_logger().info('odometry is not available', throttle_duration_sec=1)
            return False
        if not self.trajectory:
            self.get_logger().info('trajectory is not available', throttle_duration_sec=1)
            return False
        if not self.costmap:
            self.get_logger().info('costmap is not available', throttle_duration_sec=1)
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = MppiControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()