'''
File: vns_framework_for_optimize_z.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
design the vns framework specially for optimize z variable.
----------
Author: 626
Created Date: 2023.01.03
'''


import sys
sys.path.append("..")
import numpy as np
from tqdm import trange, tqdm
from common_algorithm_by_gurobi import commonAlgorithmByGurobi
from Variable import Variable
from TRA_utils import *
from generate_instances.Integrated_Instance import Instance
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
import utils
import operators_for_z

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
        init_variable = self.input_variable # 构建init variable
        update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT'] # choose update's variable
        variable_dict, is_solved = get_variable_from_solved_model(Variable=init_variable, update_variable_list=update_variable_list, model=init_model)
        # if init variable is infeasible, change the key variable(x\y\z) to get feasible variable
        if is_solved:
            pass
        # update each variable value
        init_variable.set_auxiliary_variable(variable_dict)
        init_variable.set_time_variable(variable_dict)

        return init_variable


                
    def cal_objective_and_variable(self, variable):
        """ calculate the objective of the variable 
        
        Args:
            problem (class): instance;
            variable (class): x\y\z variable.
        
        Returns:
            variable_update (class): the update variable after solving by common algorithm;
        """
        solver = commonAlgorithmByGurobi(self.problem, variable) # init the solver
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
            # 计算cur_variable的目标函数值
            cur_variable = self.cal_objective_and_variable(cur_variable)
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
                        # print('local optimal, break out, iterated {} times'.format(step))
                        break
                # 记录在这个neighborhood中连续多少次没有找到优化的解了
                non_improve_count += 1
            # 更新迭代次数
            if non_improve_count >= self.non_improve_count:
                # 如果连续多少次都没有更新了，不要再优化浪费时间了，直接去优化其它的key variable
                break
            else:
                cur_iter_num += 1

if __name__ == "__main__":
    # create instance
    w_num = 10
    l_num = 10
    bins_num = 5
    robot_num = 5
    picking_station_num = 5
    orders_num = 5
    instance = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    # 获取初始解
    input_variable = Variable(instance)
    picking_alg = NNH_heuristic_algorithm(instance)
    picking_solution = picking_alg.NNH_main()
    sorting_solution = [np.random.randint(instance.P) for _ in range(instance.O)]
    x_val, y_val, z_val = utils.solution_transfer(instance, picking_solution, sorting_solution)
    variable_dict = {
            'x' : x_val,
            'y' : y_val,
            'z' : z_val
        }
    input_variable.set_x_variable(variable_dict)
    input_variable.set_y_variable(variable_dict)
    input_variable.set_z_variable(variable_dict)
    iter_num = 10
    non_improve_count = 5
    operators_list = [operators_for_z.Relocate(instance=instance, k=1)]
    # 构建优化z_vns的优化器
    z_vns = VNS(problem=instance, input_variable=input_variable, iter_num=iter_num, non_improve_count=non_improve_count, operators_list=operators_list)
    # 优化求解
    z_vns.run()

                
                




