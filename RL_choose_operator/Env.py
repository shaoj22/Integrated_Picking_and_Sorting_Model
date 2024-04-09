'''
File: Env.py
Project: RL_choose_operator
File Created: Thursday, 21st March 2024 3:04:18 pm
Author: limingzhe05 (lmz22@mails.tsinghua.edu.cn)
'''

import sys
sys.path.append('..')
import gymnasium as gym
from metaheuristic_algorithm.Integrated_ALNS import ALNS
import numpy as np
import tqdm


class ALNSToLearn(ALNS):
    def __init__(self, *args, **kwargs):
        super(ALNSToLearn, self).__init__(*args, **kwargs)

    def reset(self, instance=None):
        super(ALNSToLearn, self).reset()
        # 重置算例
        if instance is not None:
            self.instance = instance
            self.set_operators_list(instance)
        # 重置选算子相关指标
        self.cur_solution = self.solution_init() 
        self.cur_obj, self.cur_model_obj = self.cal_objective(self.cur_solution)
        self.best_solution = self.cur_solution
        self.best_obj = self.cur_obj
        self.best_model_obj = self.cur_model_obj
        self.temperature = self.max_temp
        self.step = 0
        self.pbar = tqdm.tqdm(range(self.iter_num), desc="ALNS Iteration")
        self.solution_accept_state = -1
    
    def single_run(self, action):
        break_opt_i, repair_opt_i = action
        new_solution = self.get_neighbour(self.cur_solution, break_opt_i, repair_opt_i)
        new_obj, new_model_obj = self.cal_objective(new_solution)
        # obj: minimize the total distance 
        if new_obj < self.best_obj:
            # 新解超出最优解
            self.best_solution = new_solution
            self.best_obj = new_obj
            if self.best_model_obj >= new_model_obj:
                self.best_model_obj = new_model_obj
            self.cur_solution = new_solution
            self.cur_obj = new_obj
            self.cur_model_obj = new_model_obj
            self.break_operators_scores[break_opt_i] += self.sigma1
            self.break_operators_steps[break_opt_i] += 1
            self.repair_operators_scores[repair_opt_i] += self.sigma1
            self.repair_operators_steps[repair_opt_i] += 1
            self.solution_accept_state = 0  # 记录解的接受状态
        elif new_obj < self.cur_obj: 
            # 新解超出当前解
            self.cur_solution = new_solution
            self.cur_obj = new_obj
            if self.best_model_obj >= new_model_obj:
                self.best_model_obj = new_model_obj
            self.break_operators_scores[break_opt_i] += self.sigma2
            self.break_operators_steps[break_opt_i] += 1
            self.repair_operators_scores[repair_opt_i] += self.sigma2
            self.repair_operators_steps[repair_opt_i] += 1
            self.solution_accept_state = 1
        elif np.random.random() < self.SA_accept((new_obj-self.cur_obj)/(self.cur_obj+1e-10), self.temperature): # percentage detaC
            # 新解被模拟退火接受
            self.cur_solution = new_solution
            self.cur_obj = new_obj
            if self.best_model_obj >= new_model_obj:
                self.best_model_obj = new_model_obj
            self.break_operators_scores[break_opt_i] += self.sigma3
            self.break_operators_steps[break_opt_i] += 1
            self.repair_operators_scores[repair_opt_i] += self.sigma3
            self.repair_operators_steps[repair_opt_i] += 1
            self.solution_accept_state = 2
        else:
            self.solution_accept_state = 3
        # reset operators weights
        if self.step % self.adaptive_period == 0: 
            self.reset_operators_scores()
        # update SA temperature
        self.temperature = self.temperature_update(self.temperature, self.step)
        # record
        if self.cur_obj < 10000:
            self.obj_iter_process.append(self.cur_obj)
        else:
            self.obj_iter_process.append(10000)
        if self.step == 500:
            self.obj_of_500 = self.cur_obj

        self.pbar.set_postfix({
            "best_obj" : self.best_obj, 
            "cur_obj" : self.cur_obj, 
            "temperature" : self.temperature,
            "best_model_obj" : self.best_model_obj,
            "cur_model_obj" : self.cur_model_obj
        })
        self.pbar.update(1)
        self.step += 1

        # 达到终止条件，输出True，否则输出False
        if self.step >= self.iter_num:
            self.pbar.close()
            return True
        else:
            return False
    

class ALNSGymEnv(gym.Env): 
    def __init__(self, instance_list, iter_num, static_flag=False):
        assert len(instance_list) > 0, "算例列表instance_list不能为空！"
        self.instance_list = instance_list
        self.alns = ALNSToLearn(self.instance_list[0], iter_num)
        self.static_flag = static_flag  # 是否不断更新算例
        self.operator_pair_list = [(i, j) for i in range(len(self.alns.break_operators_list)) for j in range(len(self.alns.repair_operators_list)) if self.alns.break_operators_list[i].type == self.alns.repair_operators_list[j].type]
        self.action_num = len(self.operator_pair_list)
        self.action_space = gym.spaces.Discrete(self.action_num)
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(8+self.action_num,),  # 12 features
            dtype=np.float32
        )
        self.reward_list = [5, 3, 1, 0]  # 奖励对应于解的各个接受状态
        self.reset()
    
    def reset(self, seed=None, instance=None):
        if self.static_flag:
            self.alns.reset()
        elif instance is None:
            np.random.seed(seed)
            new_instance = np.random.choice(self.instance_list)
            self.alns.reset(new_instance)
        else:
            self.alns.reset(instance) 
        # 重置状态相关量
        self.s_reduced_cost = 0
        self.s_cost_from_min = 0
        self.s_cost = self.alns.cur_obj
        self.s_min_cost = self.alns.cur_obj
        self.s_temp = self.alns.temperature
        self.s_cs = self.alns.cooling_rate
        self.s_no_improvement = 0
        self.s_index_step = 0
        self.s_last_action_sign = 0
        self.s_last_action = [0] * self.action_num
        # 重置对结果的记录
        self.last_obj = self.alns.cur_obj
        self.last_best_obj = self.alns.cur_obj 
        self.no_improvement_count = 0
        self.last_action = -1
        return self.get_state(), {}
    
    def get_state(self):
        state = [
            self.s_reduced_cost, 
            self.s_cost_from_min, 
            self.s_min_cost,
            self.s_temp,
            self.s_cs,
            self.s_no_improvement,
            self.s_index_step,
            self.s_last_action_sign
        ] + self.s_last_action
        state = np.array(state, dtype=np.float32)
        return state
    
    def step(self, action):
        # 推进ALNS计算
        done = self.alns.single_run(self.operator_pair_list[action])
        # 计算状态信息
        self.s_reduced_cost = self.alns.cur_obj - self.last_obj
        self.s_cost_from_min = self.alns.cur_obj - self.alns.best_obj
        self.s_cost = self.alns.cur_obj
        self.s_min_cost = self.alns.best_obj
        self.s_temp = self.alns.temperature
        self.s_cs = self.alns.cooling_rate
        if self.alns.cur_obj < self.last_obj:
            self.no_improvement_count = 0
        else:
            self.no_improvement_count += 1
        self.s_no_improvement = self.no_improvement_count
        self.s_index_step = self.alns.step
        self.s_last_action_sign = 1 if self.alns.cur_obj < self.last_obj else 0
        self.s_last_action = [0] * self.action_num
        if self.last_action != -1:
            self.s_last_action[self.last_action] = 1
        state = self.get_state()
        # 计算奖励
        reward = self.reward_list[self.alns.solution_accept_state]
        # 更新结果记录
        self.last_action = action
        self.last_obj = self.alns.cur_obj
        self.last_best_obj = self.alns.best_obj
        terminated = done
        truncated = False
        return state, reward, terminated, truncated, {}

