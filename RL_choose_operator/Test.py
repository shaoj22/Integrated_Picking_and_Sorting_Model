'''
File: Test.py
Project: RL_choose_operator
File Created: Tuesday, 9th April 2024 10:36:40 am
Author: limingzhe05 (lmz22@mails.tsinghua.edu.cn)
Description: 测试模型在ALNS中的效果，以目标值为观测指标
'''

import sys
sys.path.append('..')
import os
from generate_instances import Integrated_Instance
from generate_instances.generate_instances import generate_medium_instances
from RL_choose_operator.Env import ALNSGymEnv
from stable_baselines3 import PPO
from collections import defaultdict
import time
import json


class ChooseOperatorWithModel:
    def name(self):
        return 'ChooseOperatorWithModel'

    def predict(self, model, env, obs):
        action, _states = model.predict(obs)
        return action

class ChooseOperatorWithALNS:
    def name(self):
        return 'ChooseOperatorWithALNS'

    def predict(self, model, env, obs):
        break_opt_i, repair_opt_i = env.alns.choose_operator()
        action = env.operator_pair_list.index((break_opt_i, repair_opt_i)) 
        return action


class Tester:
    def __init__(self, instance_list, iter_num, model_path, result_dir):
        self.instance_list = instance_list
        self.env = ALNSGymEnv(instance_list, iter_num)
        self.model = PPO.load(model_path, env=self.env)
        self.result_dir = result_dir
    
    def test(self):
        alg_list = [ChooseOperatorWithModel(), ChooseOperatorWithALNS()]
        test_result = {alg.name(): defaultdict(list) for alg in alg_list}
        for instance in self.instance_list:
            for alg in alg_list:
                start_time = time.time()
                obs, info = self.env.reset(instance=instance)
                done = False
                iter_obj_list = []
                while not done:
                    action = alg.predict(self.model, self.env, obs)
                    obs, rewards, terminated, truncated, info = self.env.step(action)
                    done = terminated or truncated
                    iter_obj_list.append(self.env.alns.best_obj)
                test_result[alg.name()]["obj"].append(self.env.alns.best_obj)
                test_result[alg.name()]["timecost"].append(time.time() - start_time)
                test_result[alg.name()]["iter_obj_list"].append(iter_obj_list)
        # 保存结果
        with open(self.result_dir + '/test_result.json', 'w') as f:
            json.dump(test_result, f) 
        return test_result


if __name__ == '__main__':
    cur_dir = os.path.dirname(__file__)
    # instance_params_list = [eval(params_str) for params_str in open(cur_dir + '/data/test_instances.txt').readlines()]
    instance_params_list = [eval(params_str) for params_str in open(cur_dir + '/data/train_instances.txt').readlines()]
    instance_list = [Integrated_Instance.Instance(*params) for params in instance_params_list]
    tester = Tester(instance_list, 1000, cur_dir + '\log\sb3\ppo-iter1000\model.zip', cur_dir + '/result')
    tester.test()

        


