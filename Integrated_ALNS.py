'''
File: Integrated_ALNS.py
Project: Integrated_Picking---Sorting_Model
Description: Solving integrated model with Adaptive Large Neighbourhood Search algorithm
File Created: Saturday, 6th May 2023 10:11:26 am
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''

import numpy as np
import matplotlib.pyplot as plt
import math
import Integrated_Instance
import utils
import time
import tqdm
import copy
from integrated_Operators import *
from NNH_heuristic_algorithm import NNH_heuristic_algorithm


class ALNS_base:
    """
    Base class of ALNS algorithm
        ALNS算法的通用框架，实际使用时，需要继承该类，并实现以下方法：
            1. get_operators_list: 返回一个operator的list，每个operator都是一个类，实现了get方法 
            2. solution_init: 返回一个初始解
            3. cal_objective: 计算解的目标函数值
    """
    def __init__(self, iter_num):
        self.iter_num = iter_num
        
        # set params
        ## 1. ALNS params
        self.adaptive_period = 10000
        self.sigma1 = 2
        self.sigma2 = 1
        self.sigma3 = 0.1
        ## 2. SA params
        self.max_temp = 1
        self.min_temp = 0
        self.cooling_rate = 0.98
        self.cooling_period = 10000
    
    # to be implemented in subclass
    def set_operators_list(self):
        self.break_operators_list = []
        self.repair_operators_list = []
        raise NotImplementedError

    # to be implemented in subclass
    def solution_init(self):
        raise NotImplementedError
    
    # to be implemented in subclass
    def cal_objective(self):
        raise NotImplementedError 

    def reset(self):
        self.reset_operators_scores()
        self.obj_iter_process = []
    
    def reset_operators_scores(self):
        self.break_operators_scores = np.ones(len(self.break_operators_list))
        self.repair_operators_scores = np.ones(len(self.repair_operators_list))
        self.break_operators_steps = np.ones(len(self.break_operators_list))
        self.repair_operators_steps = np.ones(len(self.repair_operators_list))
    
    def SA_accept(self, detaC, temperature):
        return math.exp(-detaC / temperature)

    def temperature_update(self, temperature, step):
        if step % self.cooling_period == 0: # update temperature by static steps
            temperature *= self.cooling_rate
        temperature = max(self.min_temp, temperature)
        return temperature

    def choose_operator(self):
        break_weights = self.break_operators_scores / self.break_operators_steps
        repair_weights = self.repair_operators_scores / self.repair_operators_steps
        break_prob = break_weights / sum(break_weights)
        repair_prob = repair_weights / sum(repair_weights)
        break_opt_i = np.random.choice(range(len(self.break_operators_list)), p=break_prob)
        repair_opt_i = np.random.choice(range(len(self.repair_operators_list)), p=repair_prob)
        return break_opt_i, repair_opt_i
    
    def get_neighbour(self, solution, break_opt_i, repair_opt_i):
        solution = copy.deepcopy(solution)
        break_p_list, break_d_list = self.break_operators_list[break_opt_i].set(solution)
        self.repair_operators_list[repair_opt_i].set(solution, break_p_list, break_d_list)
        return solution

    def show_process(self):
        y = self.process
        x = np.arange(len(y))
        plt.plot(x, y)
        plt.title("Iteration Process of ALNS")
        plt.xlabel("Iteration")
        plt.ylabel("Objective")
        plt.show()

    def run(self):
        self.reset()
        cur_solution = self.solution_init() 
        cur_obj = self.cal_objective(cur_solution)
        self.best_solution = cur_solution
        self.best_obj = cur_obj
        temperature = self.max_temp
        for step in tqdm.trange(self.iter_num, desc="ALNS Iteration", postfix={
                "best_obj" : self.best_obj, 
                "cur_obj" : cur_obj, 
                "temperature" : temperature
            }):
            break_opt_i, repair_opt_i = self.choose_operator()
            new_solution = self.get_neighbour(cur_solution, break_opt_i, repair_opt_i) 
            new_obj = self.cal_objective(new_solution)
            # obj: minimize the total distance 
            if new_obj < self.best_obj:
                self.best_solution = new_solution
                self.best_obj = new_obj
                cur_solution = new_solution
                cur_obj = new_obj
                self.break_operators_scores[break_opt_i] += self.sigma1
                self.break_operators_steps[break_opt_i] += 1
                self.repair_operators_scores[repair_opt_i] += self.sigma1
                self.repair_operators_steps[repair_opt_i] += 1
            elif new_obj < cur_obj: 
                cur_solution = new_solution
                cur_obj = new_obj
                self.break_operators_scores[break_opt_i] += self.sigma2
                self.break_operators_steps[break_opt_i] += 1
                self.repair_operators_scores[repair_opt_i] += self.sigma2
                self.repair_operators_steps[repair_opt_i] += 1
            elif np.random.random() < self.SA_accept((new_obj-cur_obj)/cur_obj, temperature):
                cur_solution = new_solution
                cur_obj = new_obj
                self.break_operators_scores[break_opt_i] += self.sigma3
                self.break_operators_steps[break_opt_i] += 1
                self.repair_operators_scores[repair_opt_i] += self.sigma3
                self.repair_operators_steps[repair_opt_i] += 1
            # reset operators weights
            if step % self.adaptive_period == 0: 
                self.reset_operators_scores()
            # update SA temperature
            temperature = self.temperature_update(temperature, step)
            # record
            self.obj_iter_process.append(cur_obj)
        return self.best_solution, self.best_obj


class ALNS(ALNS_base):
    """
    ALNS algorithm for integrated picking and sorting problem 
    """
    def __init__(self, instance, iter_num):
        super().__init__(iter_num)
        self.instance = instance
        self.set_operators_list()
    
    def set_operators_list(self):
        self.break_operators_list = [
            PickingRandomBreak(instance),
        ]
        self.repair_operators_list = [
            PickingRandomRepair(instance), 
            # PickingGreedyRepair(instance), 
        ]
    
    def solution_init(self):
        picking_alg = NNH_heuristic_algorithm(self.instance)
        picking_solution = picking_alg.NNH_main()
        sorting_solution = [np.random.randint(self.instance.P) for _ in range(self.instance.O)]
        solution = {
            "picking" : picking_solution,
            "sorting" : sorting_solution
        }
        return solution
    
    def cal_objective(self, solution):
        picking_solution = solution["picking"]
        sorting_solution = solution["sorting"]
        integrated_instance = self.instance
        obj, info = utils.efficient_integrated_evaluate(integrated_instance, picking_solution, sorting_solution)
        self.obj_info = info
        return obj
        
    def test_run(self):
        cur_solution = self.solution_init() 
        cur_obj = self.cal_objective(cur_solution)
        return cur_solution, cur_obj

if __name__ == "__main__":
    # create instance
    w_num = 5
    l_num = 5
    bins_num = 10
    robot_num = 4
    picking_station_num = 5
    orders_num = 2
    instance = Integrated_Instance.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    # run algorithm
    alg = ALNS(instance, iter_num=10000)
    start = time.time()
    solution, obj = alg.test_run()
    time_cost1 = time.time() - start

    # test
    start = time.time()
    x_val, y_val, z_val = utils.solution_transfer(instance, solution["picking"], solution["sorting"])
    true_obj1 = utils.picking_integrated_evaluate(instance, x_val)
    true_obj2, info = utils.integrated_evaluate(instance, x_val, y_val, z_val)
    time_cost2 = time.time() - start
    print("\nbest_obj = {}, time_cost = {}\nbest_solution: {}".format(obj, time_cost1, solution))
    print("true_obj1 = {}".format(true_obj1))
    print("true_obj2 = {}, time_cost = {}".format(true_obj2, time_cost2))
    print()



 
    