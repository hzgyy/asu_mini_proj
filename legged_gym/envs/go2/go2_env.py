from legged_gym.envs.base.legged_robot import LeggedRobot

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil
import torch
from legged_gym.utils.terrain import Terrain

class go2Robot(LeggedRobot):

    def compute_observations(self):
        return super().compute_observations()
    
    def compute_reward(self):
        return super().compute_reward()

    def _reset_dofs(self, env_ids):
        """ Resets DOF position and velocities of selected environmments
        Positions are randomly selected within 0.5:1.5 x default positions.
        Velocities are set to zero.

        Args:
            env_ids (List[int]): Environemnt ids
        """
        self.dof_pos[env_ids] = self.default_dof_pos
        # change to small randomization
        # self.dof_pos[env_ids] = self.default_dof_pos * torch_rand_float(0.25, 1, (len(env_ids), self.num_dof), device=self.device)
        self.dof_vel[env_ids] = 0.

        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_dof_state_tensor_indexed(self.sim,
                                              gymtorch.unwrap_tensor(self.dof_state),
                                              gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))

    def _reset_root_states(self, env_ids):
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, 7:13] = 0
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
    
    def compute_observations(self):
        """ Computes observations
        """
        self.obs_buf = torch.cat((  self.base_lin_vel * self.obs_scales.lin_vel,
                                    self.base_ang_vel  * self.obs_scales.ang_vel,
                                    self.projected_gravity,
                                    # self.commands[:, :3] * self.commands_scale,
                                    #change to relative position
                                    self.base_pos - self.base_init_state[:3],
                                    self.heading.unsqueeze(-1),
                                    (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                                    # (self.dof_pos) * self.obs_scales.dof_pos,   #change to absolute value
                                    self.dof_vel * self.obs_scales.dof_vel,
                                    self.actions
                                    ),dim=-1)
        # self.privileged_obs_buf = torch.cat((
        #                             self.dof_pos,   #change to absolute value
        #                             self.dof_vel,
        #                             ),dim=-1)
        # add perceptive inputs if not blind
        # add noise if needed
        if self.add_noise:
            self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec

    def _reward_tracking_pos(self):
        x_error = torch.square(self.base_pos[:,0]-3)
        y_error1 = torch.square(self.base_pos[:,1])
        y_error = torch.square(self.base_pos[:,1]-3)
        reward_phase_1 = torch.exp(-x_error/self.cfg.rewards.tracking_sigma) - y_error1
        reward_phase_2 = torch.where(x_error<0.25, torch.exp(-y_error/self.cfg.rewards.tracking_sigma),torch.zeros_like(y_error))
        reward = reward_phase_1 + reward_phase_2*2
        return reward
    
    def _reward_tracking_heading(self):
        x_error = torch.square(self.base_pos[:,0]-3)
        desired_y = torch.where(x_error<0.25, 3,0)
        desired_heading = torch.atan2(desired_y-self.base_pos[:,1],3-self.base_pos[:,0])
        reward = -torch.square(desired_heading-self.heading)
        return reward


    
