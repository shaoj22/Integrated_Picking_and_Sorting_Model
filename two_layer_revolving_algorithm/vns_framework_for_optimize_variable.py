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
from Variable import Variable
from TRA_utils import *
from generate_instances.Integrated_Instance import Instance
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from common_algorithm_by_gurobi import commonAlgorithmByGurobi
import utils
import operators_for_x, operators_for_z


class VNS:
    def __init__(self, problem=None, input_all_solution=None, input_optimize_variable=None, iter_num=1000, non_improve_count=100, operators_list=None):
        """ input model's solution and use vns framework to optimize the solution.

        Args:
            problem (class): the instance of the input problem.
            input_all_solution ({}): input solution, including "x"/"y"/"z".
            input_optimize_variable (str): input solution after optimized, maybe "x" "y" "z".
            iter_num (int): iter num for the vns to run.
            non_improve_count (int): more than this count, the process will be closed.
            operators_list ([]): list include the operators for optimize the solution.

        Returns:
            self.best_solution ({}): the best solution after this optimize by vns.
        """

        # input parameters
        self.problem = problem
        self.input_all_solution = input_all_solution # 包含了xyz
        self.input_optimize_variable = input_optimize_variable # 当前需要优化的变量名称
        self.iter_num = iter_num
        self.operators_list = operators_list
        # vns parameters
        self.non_improve_count = non_improve_count
        # solution parameters
        self.best_solution = None

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

    def cal_objective(self, cur_solution):
        """ calculate the objective of the variable 
        
        Args:
            problem (class): instance;
            cur_solution (List): x\y\z cur_solution's code.
        
        Returns:
            cur_solution_update (class): the update cur_solution after solving by common algorithm;
        """
        # use LP algorithm to cal the objective, speed slow.

        # solver = commonAlgorithmByGurobi(self.problem, variable) # init the solver
        # model = solver.run_gurobi_model() # run the solver
        # update_variable_list = ['a1', 'b1', 'c1', 'd1', 'Q', 'passX', 'f', 'tos', 'toe', 'I', 'Ta', 'Ts', 'Te', 'T', 'FT'] # need to update variable's name 
        # variable_update = get_variable_from_solved_model(variable, update_variable_list, model) # get the update variable

        # use fitness to cal the objective, speed fast.
        if self.input_optimize_variable == "x":
            picking_solution = cur_solution
            sorting_solution = self.input_all_solution["z"]
        else:
            picking_solution = self.input_all_solution["x"]
            sorting_solution = cur_solution
        cur_obj, cur_info = utils.efficient_integrated_evaluate(self.problem, picking_solution, sorting_solution)

        return cur_obj

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
        # 调用common algorithm to get init solution：maybe do not need.
        # init_variable = self.solution_init() # init each variable value
        # 初始化全局最优解和当前最优解
        self.best_solution = self.input_all_solution[self.input_optimize_variable]
        self.best_obj = self.cal_objective(self.best_solution)
        cur_solution = self.input_all_solution[self.input_optimize_variable]
        non_improve_count = 0 # 多次迭代解未更新
        cur_iter_num = 0 # 当前迭代次数
        # 初始化邻域结构
        operator_k = 0 # 记录当前操作的operator index
        neighborhood = self.get_neighborhood(self.best_solution, operator=self.operators_list[0]) # 获取第0个operator的邻域
        # main framework
        while cur_iter_num <= self.iter_num:
            # 从当前的neighborhood中选择一个解
            cur_solution_index = self.choose_neighborhood(neighborhood) # 当前解的index
            cur_solution = neighborhood[cur_solution_index] # 当前解的solution
            # 计算cur_solution的目标函数值
            cur_obj = self.cal_objective(cur_solution)
            # 更新解
            if self.best_obj > cur_obj:
                self.operators_list.insert(0, self.operators_list.pop(operator_k)) # 把当前operator放在list的第一个位置
                self.best_solution = cur_solution
                neighborhood = self.get_neighborhood(self.best_solution, operator=self.operators_list[0]) # 获取第0个operator的邻域
                non_improve_count = 0
            else:
                neighborhood.pop(cur_solution_index) # 当前解差所以去除
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

            # print
            print("input_optimize_variable:{}, cur_iter_num:{},cur_obj:{},best_obj:{},non_improve_count:{}".format(self.input_optimize_variable, cur_iter_num, cur_obj, self.best_obj, non_improve_count))

        return self.best_solution


if __name__ == "__main__":
    # create instance
    w_num = 10
    l_num = 10
    bins_num = 20
    robot_num = 5
    picking_station_num = 5
    orders_num = 20
    instance = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    # 获取初始解
    input_variable = Variable(instance)
    picking_alg = NNH_heuristic_algorithm(instance)
    picking_solution = picking_alg.NNH_main()
    sorting_solution = [np.random.randint(instance.P) for _ in range(instance.O)]
    # x_val, y_val, z_val = utils.solution_transfer(instance, picking_solution, sorting_solution)
    # variable_dict = {
    #         'x' : x_val,
    #         'y' : y_val,
    #         'z' : z_val
    #     }
    # input_variable.set_x_variable(variable_dict)
    # input_variable.set_y_variable(variable_dict)
    # input_variable.set_z_variable(variable_dict)

    input_all_solution = {}
    input_all_solution['x'] = picking_solution
    input_all_solution['z'] = sorting_solution
    iter_num = 1000
    non_improve_count = 10
    z_operators_list = [operators_for_z.Relocate(instance=instance, k=1)]

    # 构建优化z_vns的优化器
    z_vns = VNS(problem=instance, input_all_solution=input_all_solution, input_optimize_variable="z", iter_num=iter_num, non_improve_count=non_improve_count, operators_list=z_operators_list)
    # 优化求解
    best_solution = z_vns.run()
    print(input_all_solution['z'])
    print(best_solution)

    # # 构建x_vns的优化器
    # x_operators_list = [operators_for_x.Relocate(instance=instance, k=1)]
    # x_vns = VNS(problem=instance, input_all_solution=input_all_solution, input_optimize_variable="x", iter_num=iter_num, non_improve_count=non_improve_count, operators_list=x_operators_list)

                
                




