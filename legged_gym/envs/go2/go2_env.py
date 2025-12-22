from legged_gym.envs.base.legged_robot import LeggedRobot

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil
import torch
from legged_gym.utils.terrain import Terrain
from legged_gym import LEGGED_GYM_ROOT_DIR
import os

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

    # def post_physics_step(self):
    #     """ check terminations, compute observations and rewards
    #         calls self._post_physics_step_callback() for common computations
    #         calls self._draw_debug_vis() if needed
    #     """
    #     self.gym.refresh_actor_root_state_tensor(self.sim)
    #     self.gym.refresh_net_contact_force_tensor(self.sim)
    #
    #     self.episode_length_buf += 1
    #     self.common_step_counter += 1
    #
    #     # prepare quantities
    #     self.base_pos[:] = self.root_states[:self.num_envs, 0:3]
    #     self.base_quat[:] = self.root_states[:self.num_envs, 3:7]
    #     self.rpy[:] = get_euler_xyz_in_tensor(self.base_quat[:])
    #     self.base_lin_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:self.num_envs, 7:10])
    #     self.base_ang_vel[:] = quat_rotate_inverse(self.base_quat, self.root_states[:self.num_envs, 10:13])
    #     self.projected_gravity[:] = quat_rotate_inverse(self.base_quat, self.gravity_vec)
    #     #add more
    #     forward = quat_apply(self.base_quat, self.forward_vec)
    #     self.heading = torch.atan2(forward[:, 1], forward[:, 0])
    #     self.relative_pos[:] = self.root_states[:self.num_envs, 0:3] - self.env_origins[:]
    #     self.x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
    #     self.y_error = torch.square(self.relative_pos[:,1]-self.cfg.env.desired_y)
    #
    #
    #     self._post_physics_step_callback()
    #
    #     # compute observations, rewards, resets, ...
    #     self.check_termination()
    #     self.compute_reward()
    #     # assert False, self.reset_buf
    #     env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
    #     self.reset_idx(env_ids)
    #
    #     if self.cfg.domain_rand.push_robots:
    #         self._push_robots()
    #
    #     self.compute_observations() # in some cases a simulation step might be required to refresh some obs (for example body positions)
    #
    #     self.last_actions[:] = self.actions[:]
    #     self.last_dof_vel[:] = self.dof_vel[:]
    #     self.last_root_vel[:] = self.root_states[:self.num_envs, 7:13]

    def _reset_root_states(self, env_ids):
        self.root_states[env_ids] = self.base_init_state
        self.root_states[env_ids, :3] += self.env_origins[env_ids]
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
                                    self.relative_pos,
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

    def _update_terrain_curriculum(self,env_ids):
        if not self.init_done:
            return
        

    def _create_envs(self):
        """ Creates environments:
             1. loads the robot URDF/MJCF asset,
             2. For each environment
                2.1 creates the environment, 
                2.2 calls DOF and Rigid shape properties callbacks,
                2.3 create actor with these properties and add them to the env
             3. Store indices of different bodies of the robot
        """
        asset_path = self.cfg.asset.file.format(LEGGED_GYM_ROOT_DIR=LEGGED_GYM_ROOT_DIR)
        asset_root = os.path.dirname(asset_path)
        asset_file = os.path.basename(asset_path)

        asset_options = gymapi.AssetOptions()
        asset_options.default_dof_drive_mode = self.cfg.asset.default_dof_drive_mode
        asset_options.collapse_fixed_joints = self.cfg.asset.collapse_fixed_joints
        asset_options.replace_cylinder_with_capsule = self.cfg.asset.replace_cylinder_with_capsule
        asset_options.flip_visual_attachments = self.cfg.asset.flip_visual_attachments
        asset_options.fix_base_link = self.cfg.asset.fix_base_link
        asset_options.density = self.cfg.asset.density
        asset_options.angular_damping = self.cfg.asset.angular_damping
        asset_options.linear_damping = self.cfg.asset.linear_damping
        asset_options.max_angular_velocity = self.cfg.asset.max_angular_velocity
        asset_options.max_linear_velocity = self.cfg.asset.max_linear_velocity
        asset_options.armature = self.cfg.asset.armature
        asset_options.thickness = self.cfg.asset.thickness
        asset_options.disable_gravity = self.cfg.asset.disable_gravity

        robot_asset = self.gym.load_asset(self.sim, asset_root, asset_file, asset_options)
        self.num_dof = self.gym.get_asset_dof_count(robot_asset)
        self.num_bodies = self.gym.get_asset_rigid_body_count(robot_asset)
        dof_props_asset = self.gym.get_asset_dof_properties(robot_asset)
        rigid_shape_props_asset = self.gym.get_asset_rigid_shape_properties(robot_asset)

        # save body names from the asset
        body_names = self.gym.get_asset_rigid_body_names(robot_asset)
        self.dof_names = self.gym.get_asset_dof_names(robot_asset)
        self.num_bodies = len(body_names)
        self.num_dofs = len(self.dof_names)
        feet_names = [s for s in body_names if self.cfg.asset.foot_name in s]
        penalized_contact_names = []
        for name in self.cfg.asset.penalize_contacts_on:
            penalized_contact_names.extend([s for s in body_names if name in s])
        termination_contact_names = []
        for name in self.cfg.asset.terminate_after_contacts_on:
            termination_contact_names.extend([s for s in body_names if name in s])

        base_init_state_list = self.cfg.init_state.pos + self.cfg.init_state.rot + self.cfg.init_state.lin_vel + self.cfg.init_state.ang_vel
        self.base_init_state = to_torch(base_init_state_list, device=self.device, requires_grad=False)

        start_pose = gymapi.Transform()
        start_pose.p = gymapi.Vec3(*self.base_init_state[:3])

        self._get_env_origins()
        spacing = 5.0
        env_lower = gymapi.Vec3(-spacing, spacing, 0.)
        env_upper = gymapi.Vec3(-spacing, spacing, 0.)
        self.actor_handles = []
        self.envs = []
        # add obstacles
        # create table asset
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        #wall1
        wall1_dims = gymapi.Vec3(4.6, 4.6, 0.5)
        wall1_asset = self.gym.create_box(self.sim, wall1_dims.x, wall1_dims.y, wall1_dims.z, asset_options)
        # wall1_pose = torch.tensor([9.8,4.3,0.5*wall1_dims.z],device=self.device,requires_grad=False)
        wall1_pose = torch.tensor([2,2.8,0.5*wall1_dims.z],device=self.device,requires_grad=False)
        # wall1_pose = gymapi.Transform()
        # wall1_pose.p = gymapi.Vec3(8.8, 4.3, 0.5 * wall1_dims.z)
        #wall 2
        wall2_dims = gymapi.Vec3(6.6, 1, 0.5)
        wall2_asset = self.gym.create_box(self.sim, wall2_dims.x, wall2_dims.y, wall2_dims.z, asset_options)
        # wall2_pose = torch.tensor([10.8,0.5,0.5*wall2_dims.z],device=self.device,requires_grad=False)
        wall2_pose = torch.tensor([3,-1,0.5*wall2_dims.z],device=self.device,requires_grad=False)
        # wall2_pose = gymapi.Transform()
        # wall2_pose.p = gymapi.Vec3(9.8, 0.5, 0.5 * wall2_dims.z)
        #wall 3
        wall3_dims = gymapi.Vec3(1, 5.6, 0.5)
        wall3_asset = self.gym.create_box(self.sim, wall3_dims.x, wall3_dims.y, wall3_dims.z, asset_options)
        # wall3_pose = torch.tensor([13.6,3.8,0.5*wall1_dims.z],device=self.device,requires_grad=False)
        wall3_pose = torch.tensor([5.8,2.3,0.5*wall1_dims.z],device=self.device,requires_grad=False)
        # wall2_pose = gymapi.Transform()
        # wall2_pose.p = gymapi.Vec3(12.6, 3.8, 0.5 * wall2_dims.z)
        wall_assets = [wall1_asset,wall2_asset,wall3_asset]
        wall_poses = [wall1_pose,wall2_pose,wall3_pose]

        # 设置球颜色（可选）
        # sphere_geom_params = gymapi.AssetOptions()
        # sphere_geom_params.disable_gravity = True   # 不受重力影响
        # sphere_geom_params.fix_base_link = True     # 固定不动
        # # 创建 asset（球体）
        # sphere_asset = self.gym.create_sphere(self.sim,0.2, sphere_geom_params)
        # # 设定球位置
        # sphere_pose = gymapi.Transform()
        # sphere_pose.p = gymapi.Vec3(0, 0.0, 0.0)   # x,y,z 位置
        # sphere_pose.r = gymapi.Quat(0,0,0,1)         # 无旋转
        for i in range(self.num_envs):
            # create env instance
            env_handle = self.gym.create_env(self.sim, env_lower, env_upper, int(np.sqrt(self.num_envs)))
            # pos = self.env_origins[i].clone()
            # pos[:2] += torch_rand_float(-1., 1., (2,1), device=self.device).squeeze(1)
            # start_pose.p = gymapi.Vec3(*pos)
                
            rigid_shape_props = self._process_rigid_shape_props(rigid_shape_props_asset, i)
            self.gym.set_asset_rigid_shape_properties(robot_asset, rigid_shape_props)
            actor_handle = self.gym.create_actor(env_handle, robot_asset, start_pose, self.cfg.asset.name, i, self.cfg.asset.self_collisions, 0)
            dof_props = self._process_dof_props(dof_props_asset, i)
            self.gym.set_actor_dof_properties(env_handle, actor_handle, dof_props)
            body_props = self.gym.get_actor_rigid_body_properties(env_handle, actor_handle)
            body_props = self._process_rigid_body_props(body_props, i)
            self.gym.set_actor_rigid_body_properties(env_handle, actor_handle, body_props, recomputeInertia=True)
            self.envs.append(env_handle)
            self.actor_handles.append(actor_handle)

        self.feet_indices = torch.zeros(len(feet_names), dtype=torch.long, device=self.device, requires_grad=False)
        for i in range(len(feet_names)):
            self.feet_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0], self.actor_handles[0], feet_names[i])

        self.penalised_contact_indices = torch.zeros(len(penalized_contact_names), dtype=torch.long, device=self.device, requires_grad=False)
        for i in range(len(penalized_contact_names)):
            self.penalised_contact_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0], self.actor_handles[0], penalized_contact_names[i])

        self.termination_contact_indices = torch.zeros(len(termination_contact_names), dtype=torch.long, device=self.device, requires_grad=False)
        for i in range(len(termination_contact_names)):
            self.termination_contact_indices[i] = self.gym.find_actor_rigid_body_handle(self.envs[0], self.actor_handles[0], termination_contact_names[i])
        
        # table_handle = self.gym.create_actor(env_handle, table_asset, table_pose, "table", -1, 1)
        #create obstacles
        for i,env  in enumerate(self.envs):
            pos = self.env_origins[i].clone()
            for j,(asset,pose) in enumerate(zip(wall_assets,wall_poses)):
                box_pose = pose.clone()
                box_pose += pos
                start_pose.p = gymapi.Vec3(*box_pose)
                self.gym.create_actor(env, asset, start_pose, f"wall{j}", i, 1)


    # def _reward_tracking_pos(self):
    #     x_error = torch.square(self.relative_pos[:,0]-3)
    #     y_error1 = torch.square(self.relative_pos[:,1])
    #     y_error = torch.square(self.relative_pos[:,1]-3)
    #     reward_phase_1 = torch.exp(-x_error/self.cfg.rewards.tracking_sigma) - y_error1
    #     reward_phase_2 = torch.where(x_error<0.25, torch.exp(-y_error/self.cfg.rewards.tracking_sigma),torch.zeros_like(y_error))
    #     reward = reward_phase_1 + reward_phase_2*2
    #     return reward

    def check_termination(self):
        """ Check if environments need to be reset
        """
        self.reset_buf = torch.any(torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1) > 1., dim=1)
        self.reset_buf |= torch.logical_or(torch.abs(self.rpy[:,1])>1.0, torch.abs(self.rpy[:,0])>0.8)
        self.time_out_buf = self.episode_length_buf > self.max_episode_length # no terminal reward for time-outs
        self.reset_buf |= self.time_out_buf
        self.reset_buf |= self.success
        # assert False,f"{type(self.reset_buf),self.reset_buf.shape}"
        #obj
        obj_contact_buf = torch.any(torch.norm(self.obj_contact_forces[:, :, :], dim=-1) > 1., dim=1)
        self.reset_buf |= obj_contact_buf
        # print(obj_contact_buf)
        # a = torch.any(torch.norm(self.contact_forces[:, self.termination_contact_indices, :], dim=-1) > 1., dim=1)
        # b = torch.logical_or(torch.abs(self.rpy[:,1])>1.0, torch.abs(self.rpy[:,0])>0.8)
        # print(a,b,self.time_out_buf,self.success,self.reset_buf)


    def _reward_tracking_pos(self):
        # x_error = torch.abs(self.relative_pos[:,0]-self.cfg.env.desired_x)
        x_error = self.x_error
        reward_phase_1 = torch.exp(-x_error/self.cfg.rewards.tracking_sigma)
        return reward_phase_1
    
    def _reward_tracking_pos2(self):
        # x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
        # y_error = torch.square(self.relative_pos[:,1]-self.cfg.env.desired_y)
        x_error = self.x_error
        y_error = self.y_error
        y_error1 = torch.square(self.relative_pos[:,1])
        y_error2 = torch.abs(self.relative_pos[:,1])
        # reward_phase_2 = torch.where(x_error<self.cfg.env.turn_threshold, torch.exp(-y_error/self.cfg.rewards.tracking_sigma)*self.cfg.rewards.pos2_scale, - y_error1)
        reward_phase_2 = torch.where(x_error<self.cfg.env.turn_threshold, torch.exp(y_error2/2)*self.cfg.rewards.pos2_scale, - y_error1*4)
        return reward_phase_2

    # def _reward_tracking_pos(self):
    #     # x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
    #     # x_error = self.x_error
    #     reward_phase_1 = torch.abs(self.relative_pos[:,0]-self.cfg.env.desired_x)
    #     return reward_phase_1
    
    # def _reward_tracking_pos2(self):
    #     # x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
    #     # y_error = torch.square(self.relative_pos[:,1]-self.cfg.env.desired_y)
    #     # x_error = torch.abs(self.relative_pos[:,0]-self.cfg.env.desired_x)
    #     # y_error = self.y_error
    #     # y_error1 = torch.square(self.relative_pos[:,1])
    #     x_error = self.x_error
    #     y_error1 = torch.abs(self.relative_pos[:,1])
    #     y_error2 = torch.abs(self.relative_pos[:,1]-self.cfg.env.desired_y)
    #     # reward_phase_2 = torch.where(x_error<self.cfg.env.turn_threshold, torch.exp(-y_error/self.cfg.rewards.tracking_sigma)*self.cfg.rewards.pos2_scale, - y_error1)
    #     reward_phase_2 = torch.where(x_error<self.cfg.env.turn_threshold, y_error2*self.cfg.rewards.pos2_scale, y_error1+self.cfg.env.desired_y)
    #     return reward_phase_2

    
    def _reward_tracking_heading(self):
        # x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
        x_error = self.x_error
        desired_y = torch.where(x_error<self.cfg.env.turn_threshold+1, self.cfg.env.desired_y,0)
        desired_heading = torch.atan2(desired_y-self.relative_pos[:,1],self.cfg.env.desired_x-self.relative_pos[:,0])
        reward = -torch.square(desired_heading-self.heading)
        return reward

    def _reward_final(self):
        # x_error = torch.square(self.relative_pos[:,0]-self.cfg.env.desired_x)
        # y_error = torch.square(self.relative_pos[:,1]-self.cfg.env.desired_y)
        # x_error = self.x_error
        # y_error = self.y_error
        # reward = torch.zeros_like(x_error)
        # mask = (x_error + y_error) < 0.1
        # reward[mask] = 50.0
        return self.success.float()
    
    def _reward_speed(self):
        speed = torch.sum(torch.square(self.base_lin_vel[:,:2]),dim=1)
        reward = torch.where(self.x_error<self.cfg.env.turn_threshold,torch.exp(-speed*2),torch.zeros_like(speed))
        return reward
