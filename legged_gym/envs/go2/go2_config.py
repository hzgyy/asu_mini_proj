from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GO2RoughCfg( LeggedRobotCfg ):
    class init_state( LeggedRobotCfg.init_state ):
        #original
        pos = [0.0, 0.0, 0.42] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.1,   # [rad]
            'RL_hip_joint': 0.1,   # [rad]
            'FR_hip_joint': -0.1 ,  # [rad]
            'RR_hip_joint': -0.1,   # [rad]

            'FL_thigh_joint': 0.8,     # [rad]
            'RL_thigh_joint': 1.,       # [rad]
            'FR_thigh_joint': 0.8,     # [rad]
            'RR_thigh_joint': 1.,   # [rad]

            'FL_calf_joint': -1.5,   # [rad]
            'RL_calf_joint': -1.5,    # [rad]
            'FR_calf_joint': -1.5,  # [rad]
            'RR_calf_joint': -1.5,    # [rad]

            'front_dorsal_joint': 0.0,
            'back_dorsal_joint': 0.0,
        }
        # default_joint_angles = { # = target angles [rad] when action = 0.0
        #     'FL_hip_joint': 0.2,   # [rad] 0.1
        #     'RL_hip_joint': 0.2,   # [rad]
        #     'FR_hip_joint': -0.2 ,  # [rad]-0.1
        #     'RR_hip_joint': -0.2,   # [rad]

        #     'FL_thigh_joint': 1.15,     # [rad] 0.8 1.57
        #     'RL_thigh_joint': 1.15,   # [rad]   1
        #     'FR_thigh_joint': 1.15,     # [rad] 0.8
        #     'RR_thigh_joint': 1.15,   # [rad]   1

        #     'FL_calf_joint': -2.72,   # [rad] -1.5  -2.72
        #     'RL_calf_joint': -2.72,    # [rad] -1.5
        #     'FR_calf_joint': -2.72,  # [rad] -1.5
        #     'RR_calf_joint': -2.72,    # [rad] original -1.5

        #     'front_dorsal_joint': 0.0,
        #     'back_dorsal_joint': 0.0,
        # }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    # class rewards( LeggedRobotCfg.rewards ):
    #     soft_dof_pos_limit = 0.9
    #     base_height_target = 0.25
    #     class scales( LeggedRobotCfg.rewards.scales ):
    #         torques = -0.0002
    #         dof_pos_limits = -10.0

    #new robot
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 1
        base_height_target = 0.3
        max_contact_force = 150.
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -1.0
            base_height = -15.
            lin_vel_z = -1
            feet_air_time = 1.
    
    class env( LeggedRobotCfg.env ):
        num_observations = 48
        num_actions = 12
        episode_length_s = 20
            
    #standup policy
    # class rewards( LeggedRobotCfg.rewards ):
    #     class scales( LeggedRobotCfg.rewards.scales ):
    #         termination = -0.0
    #         tracking_lin_vel = 0.0
    #         tracking_ang_vel = 0.0
    #         lin_vel_z = -0
    #         lin_vel_xy = -0.2
    #         ang_vel_xy = -0.05
    #         orientation = -0.05
    #         torques = -0.0
    #         dof_vel = -0.
    #         dof_acc = -0
    #         base_height = 0.5 #-0.
    #         feet_air_time =  0.5
    #         collision = -0.1
    #         feet_stumble = -0.0 
    #         action_rate = -0.0001
    #         stand_still = -0.1

    #     only_positive_rewards = False # if true negative total rewards are clipped at zero (avoids early termination problems)
    #     tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
    #     soft_dof_pos_limit = 1. # percentage of urdf limits, values above this limit are penalized
    #     soft_dof_vel_limit = 1.
    #     soft_torque_limit = 1.
    #     base_height_target = 0.25
    #     max_contact_force = 200. # forces above this value are penalized

class GO2RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_go2'

  
