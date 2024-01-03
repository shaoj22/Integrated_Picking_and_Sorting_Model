'''
File: TRA_algorithm_framework.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
the main framework of the TRA algorithm to solve the Integrated model.
----------
Author: 626
Created Date: 2023.12.08
'''


from x_relaxed_gurobi_model import xRelaxedGurobiModel
from y_relaxed_gurobi_model import yRelaxedGurobiModel
from z_relaxed_gurobi_model import zRelaxedGurobiModel
from two_layer_revolving_algorithm.common_algorithm_by_gurobi import xyzRelaxedGurobiModel
from TRA_algorithm_framework import Variable
from TRA_utils import *
from optimize_x_by_vns import optimize_x_by_vns
from optimize_y_by_vns import optimize_y_by_vns
from optimize_z_by_vns import optimize_z_by_vns
from initialization_for_TRA import initialization_for_TRA


class TRAAlgorithmFramework:
    def __init__(self, problem=None, iter_num=5, accept_gap=0.2, iter_num_dict=None, non_improve_count_dict=None, operators_list_dict=None):
        # input parameters
        self.problem = problem

        # TRA parameters
        self.accept_gap = accept_gap
        self.need_to_optimize_variable = ['x', 'y', 'z']
        self.iter_num = iter_num
        
        # Iter parameters
        self.x_iter_num = iter_num_dict['x']
        self.y_iter_num = iter_num_dict['y']
        self.z_iter_num = iter_num_dict['z']
        self.x_non_improve_count = non_improve_count_dict['x']
        self.y_non_improve_count = non_improve_count_dict['y']
        self.z_non_improve_count = non_improve_count_dict['z']

        # operators list 
        self.x_operators_list = operators_list_dict['x']
        self.y_operators_list = operators_list_dict['y'] 
        self.z_operators_list = operators_list_dict['z']


    def sub_process_solver(self, cur_optimize_variable, input_variable):
        """ input the variable need to optimize and use the variable by vns 
        
        Args:
            cur_optimize_variable (String): 'x' or 'y' or 'z'.
            input_variable (class): current variable.
        Return:
            update_variable (class): use the vns to optimized.
        """
        if cur_optimize_variable == 'x':
            return optimize_x_by_vns(problem=self.problem, input_variable=input_variable, 
                                     iter_num=self.x_iter_num, 
                                     non_improve_count=self.x_non_improve_count, 
                                     operators_list=self.x_operators_list)
        elif cur_optimize_variable == 'y':
            return optimize_y_by_vns(problem=self.problem, input_variable=input_variable, 
                                     iter_num=self.y_iter_num, 
                                     non_improve_count=self.y_non_improve_count, 
                                     operators_list=self.y_operators_list)
        elif cur_optimize_variable == 'z':
            return optimize_z_by_vns(problem=self.problem, input_variable=input_variable, 
                                     iter_num=self.z_iter_num, 
                                     non_improve_count=self.z_non_improve_count, 
                                     operators_list=self.z_operators_list)


    def main(self):
        # 初始化Variable
        variable = Variable(self.problem)
        # use ALNS to get solution as init variable / initialization for the TRA
        init_variable = initialization_for_TRA(variable, self.problem)
        cur_variable = init_variable # 当前解
        best_variable = cur_variable # 最优解
        # 初始化循环控制参数
        stop = False
        variable_id_index = 0 # to chose the variable
        mark = 0
        iter_num = 0 # 总的迭代次数
        # iter main framework
        while not stop:
            # chose the current need to optimize variable 1/3.
            cur_optimize_variable = self.need_to_optimize_variable[variable_id_index]
            # 不优化yip
            if cur_optimize_variable == 'y':
                continue
            # 判断model的解与lower bound相比是否满足要求，若不满足则继续优化model的解
            lower_bound_of_model = 0
            if ((best_variable.FT - lower_bound_of_model)/best_variable.FT - self.accept_gap):
                # 调用对应的sub-process to optimize the variable
                cur_variable = self.sub_process_solver(cur_optimize_variable, best_variable)
            # 判断是否更新best_variable
            if (cur_variable.FT < best_variable.FT):
                best_variable = cur_variable
                mark = 0
            else:
                mark += 1
            iter_num += 1
            # 更改优化的决策变量
            if variable_id_index < 2:
                variable_id_index += 1 # 优化下一个决策变量
            else:
                variable_id_index = 0 # 旋转优化第一个决策变量
            # 判断算法终止条件
            if (mark >= 3 or iter_num >= self.max_iter_num):
                stop = True
                print("algorithm stop")
        self.best_variable = best_variable

    def runner(self):
        self.main()




