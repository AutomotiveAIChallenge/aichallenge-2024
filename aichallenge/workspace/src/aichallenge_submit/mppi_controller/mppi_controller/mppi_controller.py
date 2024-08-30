import torch
import numpy as np
from typing import Tuple
from mppi_controller.MPPI import MPPI
from mppi_controller.cost_map_tensor import CostMapTensor

import time

@torch.jit.script
def angle_normalize(x) -> torch.Tensor:
    return ((x + torch.pi) % (2 * torch.pi)) - torch.pi

class mppi_controller:
    def __init__(self, config, debug=False, device=torch.device("cuda"), dtype=torch.float32):
        self.config = config
        self.debug = debug
        # device and dtype
        if torch.cuda.is_available() and device == torch.device("cuda"):
            self._device = torch.device("cuda")
        else:
            self._device = torch.device("cpu")
        self._dtype = dtype

        self.u_min = torch.tensor(self.config["u_min"], device=self._device, dtype=self._dtype)
        self.u_max = torch.tensor(self.config["u_max"], device=self._device, dtype=self._dtype)
        self.sigmas = torch.tensor(self.config["sigmas"], device=self._device, dtype=self._dtype)

        # solver
        self.solver = MPPI(
            horizon=self.config["horizon"],
            num_samples=self.config["num_samples"],
            dim_state=4,
            dim_control=2,
            dynamics=self.dynamics,
            cost_func=self.cost_function,
            u_min=self.u_min,
            u_max=self.u_max,
            sigmas=self.sigmas,
            lambda_=self.config["lambda"],
            auto_lambda=self.config["auto_lambda"],
        )

        # model parameter
        self.delta_t = torch.tensor(self.config["delta_t"], device=self._device, dtype=self._dtype)
        self.vehicle_L = torch.tensor(self.config["vehicle_L"], device=self._device, dtype=self._dtype)
        self.V_MAX = torch.tensor(self.config["V_MAX"], device=self._device, dtype=self._dtype)
        
        # cost weights
        self.Qc = self.config["Qc"]  # contouring error cost
        self.Ql = self.config["Ql"]  # lag error cost
        self.Qv = self.config["Qv"]  # velocity cost
        self.Qo = self.config["Qo"]  # obstacle cost
        self.Qin = self.config["Qin"]  # input cost
        self.Qdin = self.config["Qdin"]  # input rate cost

        self.current_path_index = 0

        # reference indformation (tensor)
        self.reference_path: torch.Tensor = None
        self.cost_map: CostMapTensor = None

    def update(self, state: torch.Tensor, racing_center_path: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Update the controller with the current state and reference path.
        Args:
            state (torch.Tensor): current state of the vehicle, shape (4,) [x, y, yaw, v]
            racing_center_path (torch.Tensor): racing center path, shape (N, 3) [x, y, yaw]
        Returns:
            Tuple[torch.Tensor, torch.Tensor]: action sequence tensor, shape (horizon, 2) [accel, steer], state sequence tensor, shape (horizon + 1, 4) [x, y, yaw, v]
        """
        
        # reference
        self.reference_path, self.current_path_index = self.calc_ref_trajectory(
            state, racing_center_path, self.current_path_index, self.solver._horizon, DL=self.config["DL"], lookahead_distance=self.config["lookahead_distance"], reference_path_interval=self.config["reference_path_interval"]
        )

        if self.reference_path is None:
            raise ValueError("reference path, obstacle map, and lane map must be set before calling solve method.")

        # solve        
        start = time.time()
        action_seq, state_seq = self.solver.forward(state=state)
        end = time.time()
        solve_time = end - start

        if self.debug:
            print("solve time: {}".format(round(solve_time * 1000, 2)), " [ms]")

        return action_seq, state_seq
    
    def get_top_samples(self, num_samples = 300) -> Tuple[torch.Tensor, torch.Tensor]:
        return self.solver.get_top_samples(num_samples=num_samples)
    
    def set_cost_map(self, cost_map: CostMapTensor) -> None:
        self.cost_map = cost_map

    def dynamics(
        self, state: torch.Tensor, action: torch.Tensor
    ) -> torch.Tensor:
        """
        Update robot state based on differential drive dynamics.
        Args:
            state (torch.Tensor): state batch tensor, shape (batch_size, 3) [x, y, theta, v]
            action (torch.Tensor): control batch tensor, shape (batch_size, 2) [accel, steer]
            delta_t (float): time step interval [s]
        Returns:
            torch.Tensor: shape (batch_size, 3) [x, y, theta]
        """

        # Perform calculations as before
        x = state[:, 0].view(-1, 1)
        y = state[:, 1].view(-1, 1)
        theta = state[:, 2].view(-1, 1)
        v = state[:, 3].view(-1, 1)
        accel = torch.clamp(action[:, 0].view(-1, 1), self.u_min[0], self.u_max[0])
        steer = torch.clamp(action[:, 1].view(-1, 1), self.u_min[1], self.u_max[1])
        theta = angle_normalize(theta)

        dx = v * torch.cos(theta)
        dy = v * torch.sin(theta)
        dv = accel
        dtheta = v * torch.tan(steer) / self.vehicle_L

        new_x = x + dx * self.delta_t
        new_y = y + dy * self.delta_t
        new_theta = angle_normalize(theta + dtheta * self.delta_t)
        new_v = v + dv * self.delta_t
        
        # Clamp velocity
        new_v = torch.clamp(new_v, -self.V_MAX, self.V_MAX)

        result = torch.cat([new_x, new_y, new_theta, new_v], dim=1)

        return result
        

    def cost_function(self, state: torch.Tensor, action: torch.Tensor, info: dict) -> torch.Tensor:
        """
        Calculate cost function
        Args:
            state (torch.Tensor): state batch tensor, shape (batch_size, 4) [x, y, theta, v]
            action (torch.Tensor): control batch tensor, shape (batch_size, 2) [accel, steer]
        Returns:
            torch.Tensor: shape (batch_size,)
        """
        # info
        prev_action = info["prev_action"]
        t = info["t"] # horizon number

        # path cost
        # contouring and lag error of path
        ec = torch.sin(self.reference_path[t, 2]) * (state[:, 0] - self.reference_path[t, 0]) \
            -torch.cos(self.reference_path[t, 2]) * (state[:, 1] - self.reference_path[t, 1])
        el = -torch.cos(self.reference_path[t, 2]) * (state[:, 0] - self.reference_path[t, 0]) \
             -torch.sin(self.reference_path[t, 2]) * (state[:, 1] - self.reference_path[t, 1])

        path_cost = self.Qc * ec.pow(2) + self.Ql * el.pow(2)

        # velocity cost
        v = state[:, 3]
        v_target = self.reference_path[t, 3]
        velocity_cost = self.Qv * (v - v_target).pow(2)

        # compute obstacle cost from cost map
        pos_batch = state[:, :2].unsqueeze(1)  # (batch_size, 1, 2)
        obstacle_cost = self.cost_map.compute_cost(pos_batch).squeeze(1)  # (batch_size,)
        obstacle_cost = self.Qo * obstacle_cost

        # input cost
        input_cost = self.Qin * action.pow(2).sum(dim=1)
        input_cost += self.Qdin * (action - prev_action).pow(2).sum(dim=1)

        cost = path_cost + velocity_cost + obstacle_cost + input_cost

        return cost
    
    def calc_ref_trajectory(self, state: torch.Tensor, path: torch.Tensor, 
                            cind: int, horizon: int, DL=0.1, lookahead_distance=1.0, reference_path_interval=0.5
                            ) -> Tuple[torch.Tensor, int]:
        """
        Calculate the reference trajectory for the vehicle.

        Args:
            state (torch.Tensor): current state of the vehicle, shape (4,) [x, y, yaw, v]
            path (torch.Tensor): reference path, shape (N, 4) [x, y, yaw, target_v]
            cind (int): current index of the vehicle on the path
            horizon (int): prediction horizon
            DL (float): resolution of the path
            lookahead_distance (float): distance to look ahead
            reference_path_interval (float): interval of the reference path

        Returns:
            Tuple[torch.Tensor, int]: reference trajectory tensor, shape (horizon + 1, 4) [x, y, yaw, target_v], index of the vehicle on the path
        """

        def resample_path(path, DL):
            # Calculate the number of segments needed for each pair of points
            distances = torch.norm(path[1:, :2] - path[:-1, :2], dim=1)
            num_points = torch.ceil(distances / DL).to(torch.int64)

            # Create a tensor to store the new resampled path points
            total_points = num_points.sum() + 1  # Include the first point
            new_path = torch.zeros((total_points, path.shape[1]), dtype=path.dtype, device=path.device)

            # Initialize the first point
            new_path[0] = path[0]

            # Generate all new points at once
            start_idx = 1
            for i in range(len(path) - 1):
                segment_length = num_points[i].item()
                if segment_length == 0:
                    continue

                t = torch.linspace(0, 1, segment_length + 1, device=path.device)[1:]  # Skip the first point to avoid duplication
                interpolated_points = (1 - t).unsqueeze(1) * path[i] + t.unsqueeze(1) * path[i + 1]
                new_path[start_idx:start_idx + segment_length] = interpolated_points
                start_idx += segment_length

            return new_path

        # Resample the path with the specified DL
        path = resample_path(path, DL)
        ncourse = len(path)
        xref = torch.zeros((horizon + 1, state.shape[0]), dtype=state.dtype, device=state.device)

        # Calculate the nearest index to the vehicle
        ind = torch.argmin(torch.norm(path[:, :2] - state[:2], dim=1)).item()
        # Ensure the index is not less than the current index
        ind = max(cind, ind)

        # Generate the rest of the reference trajectory
        travel = lookahead_distance

        for i in range(horizon + 1):
            travel += reference_path_interval
            dind = int(round(travel / DL))

            if (ind + dind) < ncourse:
                xref[i] = path[ind + dind]
            else:
                xref[i] = path[-1]

        return xref, ind