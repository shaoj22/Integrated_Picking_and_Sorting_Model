'''
File: vns_framework_for_optimize_variable.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
design the vns framework specially for optimize x\y\z variable.
----------
Author: 626
Created Date: 2023.12.27
'''


import sys
sys.path.append("..")
import numpy as np
from tqdm import trange, tqdm
from common_algorithm_by_gurobi import commonAlgorithmByGurobi
from Variable import Variable
from TRA_utils import *


class VNS:
    def __init__(self, problem=None, input_variable=None, iter_num=1000, non_improve_count=100, operators_list=None):
        """ input model's variable and use vns framework to optimize the variable.

        Args:
            problem (class): the instance of the input problem.
            input_variable (class): variable optimized.
            iter_num (int): iter num for the vns to run.
            non_improve_count (int): more than this count, the process will be closed.
            operators_list ([]): list include the operators for optimize the variable.

        Returns:
            self.best_variable (class): the best variable after this optimize by vns.
        """

        # input parameters
        self.problem = problem
        self.input_variable = input_variable
        self.iter_num = iter_num
        self.operators_list = operators_list
        # vns parameters
        self.non_improve_count = non_improve_count
        # solution parameters
        self.best_variable = None

    def solution_init(self):
        """ use the current variable and the common algorithm to get init solution for the problem """
        solver = commonAlgorithmByGurobi(self.problem, self.input_variable) # 构建common algorithm的solver
        init_model = solver.run_gurobi_model() # run common algorithm
        init_variable = Variable(self.problem) # 构建init variable
        update_variable_list = ['x', 'y', 'z', 'a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT'] # choose update's variable
        variable_dict, is_solved = get_variable_from_solved_model(Variable=init_variable, update_variable_list=update_variable_list, model=init_model)
        # if init variable is infeasible, change the key variable(x\y\z) to get feasible variable
        if is_solved:
            pass
        
        
        
        # update each variable value
        init_variable.set_x_variable(variable_dict)
        init_variable.set_y_variable(variable_dict)
        init_variable.set_z_variable(variable_dict)
        init_variable.set_auxiliary_variable(variable_dict)
        init_variable.set_time_variable(variable_dict)

        return init_variable


                
    def cal_objective_and_variable(self, problem, variable):
        """ calculate the objective of the variable 
        
        Args:
            problem (class): instance;
            variable (class): x\y\z variable.
        
        Returns:
            variable_update (class): the update variable after solving by common algorithm;
        """
        solver = commonAlgorithmByGurobi(problem, variable) # init the solver
        model = solver.run_gurobi_model() # run the solver
        update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT'] # need to update variable's name 
        variable_update = get_variable_from_solved_model(variable, update_variable_list, model) # get the update variable

        return variable_update

    def get_neighborhood(self, variable, operator):
        """ input one operator and cur variable to get the neighborhood """
        neighborhood = operator.run(variable)

        return neighborhood
    
    def choose_neighborhood(self, neighborhood):
        """ random chose one neighborhood as cur variable """
        chosen_neighborhood = np.random.randint(len(neighborhood))

        return chosen_neighborhood

    def run(self):
        """ the main function of the vns framework: a algorithm runner """
        # 调用common algorithm to get init solution
        init_variable = self.solution_init() # init each variable value
        # 初始化全局最优解和当前最优解
        self.best_variable = init_variable
        cur_variable = init_variable
        non_improve_count = 0 # 多次迭代解未更新
        cur_iter_num = 0 # 当前迭代次数
        # 初始化邻域结构
        operator_k = 0 # 记录当前操作的operator index
        neighborhood = self.get_neighborhood(self.best_variable, operator=self.operators_list[0]) # 获取第0个operator的邻域
        # main framework
        while cur_iter_num <= self.iter_num:
            # 从当前的neighborhood中选择一个解
            cur_variable_index = self.choose_neighborhood(neighborhood) # 当前解的index
            cur_variable = neighborhood[cur_variable_index] # 当前解的variable
            # 更新解
            if self.best_variable.FT >= cur_variable.FT:
                self.operators_list.insert(0, self.operators_list.pop(operator_k)) # 把当前operator放在list的第一个位置
                self.best_variable = cur_variable
                neighborhood = self.get_neighborhood(self.best_variable, operator=self.operators_list[0]) # 获取第0个operator的邻域
                non_improve_count = 0
            else:
                neighborhood.pop(cur_variable_index) # 当前解差所以去除
                if len(neighborhood) == 0:
                    # 如果当前的neighborhood已经搜索完毕，前往搜索下一个operator的邻域
                    operator_k += 1
                    if operator_k < len(self.operators_list):
                        operator = self.operators_list[operator_k]
                        neighborhood = self.get_neighborhood(self.best_variable, operator=operator) # 获取operator的邻域
                    else:
                        print('local optimal, break out, iterated {} times'.format(step))
                        break
                # 记录在这个neighborhood中连续多少次没有找到优化的解了
                non_improve_count += 1
            # 更新迭代次数
            if non_improve_count >= self.non_improve_count:
                # 如果连续多少次都没有更新了，不要再优化浪费时间了，直接去优化其它的key variable
                break
            else:
                cur_iter_num += 1

    
                
                




