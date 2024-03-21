'''
File: Train.py
Project: RL_choose_operator
File Created: Thursday, 21st March 2024 4:16:06 pm
Author: limingzhe05 (lmz22@mails.tsinghua.edu.cn)
'''
from generate_instances import Integrated_Instance
from RL_choose_operator.Env import ALNSGymEnv
import torch.utils.tensorboard as tensorboard

# create instance
w_num = 9
l_num = 8
bins_num = 60
robot_num = 20
picking_station_num = 10
orders_num = 50
instance = Integrated_Instance.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)

#%% check env
# from stable_baselines3.common.env_checker import check_env

# env = ALNSGymEnv(instance, iter_num=100)
# check_env(env)


# #%% train
from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure

env = ALNSGymEnv(instance, iter_num=100)
model = PPO("MlpPolicy", env, verbose=1)
log_path = "RL_choose_operator/log/sb3/ppo"
logger = configure(log_path, ["tensorboard", "stdout"])
model.set_logger(logger)
model.learn(total_timesteps=100000)



