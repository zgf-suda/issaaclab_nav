# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def position_command_error_tanh(env: ManagerBasedRLEnv, std: float, tr:int,command_name: str) -> torch.Tensor:
    """Reward position tracking with tanh kernel."""
    command = env.command_manager.get_command(command_name)
    des_pos_b = command[:, :3] / std 
    distance = torch.norm(des_pos_b, dim=1)

    if env.episode_length_buf[0].cpu().numpy() < tr:
        return torch.tensor(0)
    else:
        print("distance",torch.tensor(1/tr * (1/(1+distance))))
        return torch.tensor(1/tr * (1/ (1+distance)))
    
    return 1 - torch.tanh(distance / std)


def heading_command_error_abs(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Penalize tracking orientation error."""
    command = env.command_manager.get_command(command_name)
    heading_b = command[:, 3] / 1
    if env.episode_length_buf[0].cpu().numpy() < 10:
        return torch.tensor(0)
    else:
        return torch.tensor(1/10 * (1/ (1+heading_b.abs())))
    return heading_b.abs()

    