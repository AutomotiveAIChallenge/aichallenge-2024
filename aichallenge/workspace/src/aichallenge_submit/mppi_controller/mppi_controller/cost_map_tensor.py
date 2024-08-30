import torch
from typing import Tuple

class CostMapTensor:
    def __init__(
            self, 
            cost_map: torch.Tensor, 
            cell_size: float,
            origin: Tuple[float, float] = (0.0, 0.0),
            device=torch.device("cuda"),
            dtype=torch.float32,
        ) -> None: 
        """
        cost map tensor for collision checking.
        input:
            cost_map (torch.Tensor): cost map tensor.
            cell_size (float): size(m) of each cell in the cost map.
            origin (Tuple[float, float]): origin of the cost map. (m, m)
            device: device to run the computation.
            dtype: data type of the tensor.
        """
        self.cost_map: torch.Tensor = cost_map
        self.cell_size: float = cell_size
        origin: Tuple[float, float] = origin
        self.origin_tensor = torch.tensor(origin, device=device, dtype=dtype)
        self.device = device
        self.dtype = dtype
        
    def compute_cost(self, x: torch.Tensor) -> torch.Tensor:
        """
        Check collision in a batch of trajectories.
        :param x: Tensor of shape (batch_size, traj_length, position_dim).
        :return: collsion costs on the trajectories.
        """
        assert self.cost_map is not None
        if x.device != self.device or x.dtype != self.dtype:
            x = x.to(self.device, self.dtype)

        # project to cell map
        x_occ = (x - self.origin_tensor) / self.cell_size
        x_occ = torch.round(x_occ).long().to(self.device)

        x_occ[..., 0] = torch.clamp(x_occ[..., 0], 0, self.cost_map.shape[0] - 1)
        x_occ[..., 1] = torch.clamp(x_occ[..., 1], 0, self.cost_map.shape[1] - 1)

        # collision check
        collisions = self.cost_map[x_occ[..., 1], x_occ[..., 0]]

        return collisions