import os
import numpy as np
from datetime import datetime
import sys
sys.path.append('/media/mani/Data/gyy_workspace/asu_mini_proj/unitree_rl_gym')
import isaacgym
from legged_gym.envs import *
from legged_gym.utils import get_args, task_registry
import torch

def GetCubicSplinePos(x0,v0,xf,vf,t,T):
    if t>= T:
        return xf
    d = x0
    c = v0
    a = (vf*T - 2*xf + v0*T+2*x0) / T**3
    b = (3*xf-vf*T - 2*v0*T - 3*x0) / T**2
    return a*t**3 +b*t**2 + c*t + d

def GetCubicSplineVel(x0,v0,xf,vf,t,T):
    if t>= T:
        return xf
    d = x0
    c = v0
    a = (vf*T - 2*xf + v0*T+2*x0) / T**3
    b = (3*xf-vf*T - 2*v0*T - 3*x0) / T**2
    return 3.*a*t**2 +2.*b*t + c

stand_duration = 1000
def main(args):
    env, env_cfg = task_registry.make_env(name=args.task, args=args)
    device = env.device
    action_dim = env.num_actions
    planning_joint_pos = torch.zeros(env.num_actions)
    planning_joint_vel = np.zeros(env.num_actions)
    obs,pobs = env.reset()  #obs torch.tensor (nenv,54),(nenv,28)
    #get init angles
    init_joint_pos = pobs[:,:14]
    init_joint_vel = pobs[:,14:]
    goal_joint_pos = torch.tensor([0.1,0.1,-0.1,-0.1,0.8,1,0.8,1,-1.5,-1.5,-1.5,-1.5,0,0], device=device, requires_grad=False)
    i=0
    while True:
        # action = torch.zeros((env.num_envs,env.num_actions),device=env.device)
        # obs,pobs,reward,reset,extra = env.step(action)     #tensor,tensor,tensor, extra contains information about the detailed reward
        i += 1
        # if i == 40:
        #     env.reset()
        #     i=0
        # runtime = 0
        if i <= stand_duration:
            planning_joint_pos = GetCubicSplinePos(init_joint_pos,init_joint_vel,goal_joint_pos,0,i,stand_duration)
            planning_joint_vel = GetCubicSplineVel(init_joint_pos,init_joint_vel,goal_joint_pos,0,i,stand_duration)
        env.step(planning_joint_pos)

    

if __name__ == '__main__':
    args = get_args()
    main(args)