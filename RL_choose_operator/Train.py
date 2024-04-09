'''
File: Train.py
Project: RL_choose_operator
File Created: Thursday, 21st March 2024 4:16:06 pm
Author: limingzhe05 (lmz22@mails.tsinghua.edu.cn)
'''
#%%
import sys
sys.path.append('..')
import os
from generate_instances import Integrated_Instance
from RL_choose_operator.Env import ALNSGymEnv
import torch
import torch.utils.tensorboard as tensorboard
cur_dir = os.path.dirname(__file__)

# create instance
instance_params_list = [eval(params_str) for params_str in open(cur_dir + '/data/train_instances.txt').readlines()]
instance_list = [Integrated_Instance.Instance(*params) for params in instance_params_list]

#%% check env
# from stable_baselines3.common.env_checker import check_env

# env = ALNSGymEnv(instance, iter_num=100)
# check_env(env)


# #%% train
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
import datetime

env = ALNSGymEnv(instance_list, iter_num=1000)  # 迭代次数
policy_kwargs = dict(activation_fn=torch.nn.ReLU,
                     net_arch=dict(pi=[256, 256], vf=[256, 256]))  # 网络参数
model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, gamma=0.5, learning_rate=1e-5, batch_size=64)  # 训练参数
log_path = cur_dir + "/log/sb3/ppo-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
logger = configure(log_path, ["tensorboard", "stdout"])
model.set_logger(logger)
model.learn(total_timesteps=1000000)
model.save(log_path + '/model.zip')

# %%
