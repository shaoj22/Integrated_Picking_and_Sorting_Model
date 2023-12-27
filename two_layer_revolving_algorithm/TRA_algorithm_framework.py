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


class TRAAlgorithmFramework:
    def __init__(self, iter_num=1):
        self.iter_num = iter_num

    def main(self, instance=None):
        # 初始化Variable
        init_variable = Variable(instance)
        # use ALNS to get solution
        ALNS_solution = get_solution_from_ALNS(instance)
        # set init variable using variable_update_dict_by_ALNS
        variable_update_dict_by_ALNS = get_init_variable_from_ALNS(init_variable, ALNS_solution)
        # set init variable
        init_variable.set_x_variable(variable_update_dict_by_ALNS)
        init_variable.set_y_variable(variable_update_dict_by_ALNS)
        init_variable.set_z_variable(variable_update_dict_by_ALNS)
        init_variable.set_auxiliary_variable(variable_update_dict_by_ALNS)
        init_variable.set_time_variable(variable_update_dict_by_ALNS)
        cur_variable = init_variable # 当前解
        best_variable = cur_variable # 最优解
        # 迭代求解
        for iter in range(self.iter_num):
            # 使用x_relaxed_model优化x
            x_solver = xRelaxedGurobiModel(instance, cur_variable) # 构建x求解器
            x_model = x_solver.run_gurobi_model() # 构建x模型
            update_variable_list = ['x']
            x_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, x_model)
            if is_solved:
                cur_variable.set_x_variable(x_variable_dict)  
            # 使用x_y_z_relaxed_model优化剩余的所有变量
            xyz_solver = xyzRelaxedGurobiModel(instance, cur_variable) # 构建xyz求解器
            xyz_model = xyz_solver.run_gurobi_model() # 构建xyz模型
            update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT']
            xyz_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, xyz_model)
            if is_solved:
                cur_variable.set_auxiliary_variable(xyz_variable_dict)
                cur_variable.set_time_variable(xyz_variable_dict)
            # 使用y_relaxed_model优化y
            y_solver = yRelaxedGurobiModel(instance, cur_variable) # 构建x求解器
            y_model = y_solver.run_gurobi_model() # 构建x模型
            update_variable_list = ['y']
            y_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, y_model)
            if is_solved:
                cur_variable.set_x_variable(y_variable_dict)  
            # 使用x_y_z_relaxed_model优化剩余的所有变量
            xyz_solver = xyzRelaxedGurobiModel(instance, cur_variable) # 构建xyz求解器
            xyz_model = xyz_solver.run_gurobi_model() # 构建xyz模型
            update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT']
            xyz_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, xyz_model)
            if is_solved:
                cur_variable.set_auxiliary_variable(xyz_variable_dict)
                cur_variable.set_time_variable(xyz_variable_dict)
            # 使用z_relaxed_model优化z
            z_solver = zRelaxedGurobiModel(instance, cur_variable) # 构建x求解器
            z_model = z_solver.run_gurobi_model() # 构建x模型
            update_variable_list = ['z']
            z_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, z_model)
            if is_solved:
                cur_variable.set_x_variable(z_variable_dict)  
            # 使用x_y_z_relaxed_model优化剩余的所有变量
            xyz_solver = xyzRelaxedGurobiModel(instance, cur_variable) # 构建xyz求解器
            xyz_model = xyz_solver.run_gurobi_model() # 构建xyz模型
            update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT']
            xyz_variable_dict, is_solved = get_variable_from_solved_model(cur_variable, update_variable_list, xyz_model)
            if is_solved:
                cur_variable.set_auxiliary_variable(xyz_variable_dict)
                cur_variable.set_time_variable(xyz_variable_dict)

    def runner(self, instance=None):
        self.main(instance)




