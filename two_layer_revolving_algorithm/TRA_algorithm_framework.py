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

import operators_for_x
import operators_for_y
import operators_for_z
from x_relaxed_gurobi_model import xRelaxedGurobiModel
from y_relaxed_gurobi_model import yRelaxedGurobiModel
from z_relaxed_gurobi_model import zRelaxedGurobiModel
from TRA_utils import *
from optimize_x_by_vns import optimize_x_by_vns
from optimize_y_by_vns import optimize_y_by_vns
from optimize_z_by_vns import optimize_z_by_vns
from vns_framework_for_optimize_x import VNS as vns_for_optimize_x
from vns_framework_for_optimize_y import VNS as vns_for_optimize_y
from vns_framework_for_optimize_z import VNS as vns_for_optimize_z
from initialization_for_TRA import initialization_for_TRA
from generate_instances.Integrated_Instance import Instance


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

        # best solution and best obj
        self.best_solution = None
        self.best_obj = 0


    def sub_process_solver(self, cur_optimize_variable, input_solution, input_obj):
        """ input the variable need to optimize and use the variable by vns 
        
        Args:
            cur_optimize_variable (String): 'x' or 'y' or 'z'.
            input_solution ({}): current solution.
            input_obj (int): the obj input.

        Return:
            output_solution ({}): use the vns to optimized.
        """
        output_solution = input_solution

        # 优化x
        if cur_optimize_variable == 'x':
            x_vns = vns_for_optimize_x(problem=self.problem, 
                                       picking_solution=input_solution['picking_solution'],
                                       sorting_solution=input_solution['sorting_solution'],
                                       init_obj=input_obj,
                                       iter_num=self.x_iter_num,
                                       non_improve_count=self.x_non_improve_count,
                                       operators_list=self.x_operators_list
                                       )
            new_picking_solution, new_obj = x_vns.run()
            output_solution['picking_solution'] = new_picking_solution

            return output_solution, new_obj
        
        # 优化y
        elif cur_optimize_variable == 'y':
            y_vns = vns_for_optimize_y(problem=self.problem, 
                                       picking_solution=input_solution['picking_solution'],
                                       sorting_solution=input_solution['sorting_solution'],
                                       init_obj=input_obj,
                                       iter_num=self.y_iter_num,
                                       non_improve_count=self.y_non_improve_count,
                                       operators_list=self.y_operators_list
                                       )
            new_picking_solution = y_vns.run()
            output_solution['picking_solution'] = new_picking_solution

            return output_solution

        # 优化z
        elif cur_optimize_variable == 'z':
            z_vns = vns_for_optimize_z(problem=self.problem, 
                                       picking_solution=input_solution['picking_solution'],
                                       sorting_solution=input_solution['sorting_solution'],
                                       init_obj=input_obj,
                                       iter_num=self.z_iter_num,
                                       non_improve_count=self.z_non_improve_count,
                                       operators_list=self.z_operators_list
                                       )
            new_sorting_solution, new_obj = z_vns.run()
            output_solution['sorting_solution'] = new_sorting_solution

            return output_solution, new_obj

    def main(self):
        # use greedy or ALNS algorithm to get solution as init variable / initialization for the TRA
        # 1. 获取初始解
        init_solution, init_obj = initialization_for_TRA(self.problem)
        # 2. 设置当前解为初始解
        cur_solution = init_solution
        cur_obj = init_obj
        # 3. 设置最优解为初始解
        self.best_solution = init_solution
        self.best_obj = init_obj
        # 4. 初始化循环控制参数
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
                variable_id_index += 1
                continue
            # 判断model的解与lower bound相比是否满足要求，若不满足则继续优化model的解
            lower_bound_of_model = 0
            if ((self.best_obj - lower_bound_of_model)/self.best_obj - self.accept_gap):
                # 调用对应的sub-process to optimize the variable
                cur_solution, cur_obj = self.sub_process_solver(cur_optimize_variable, self.best_solution, self.best_obj)
            # 判断是否更新best_variable
            if (cur_obj < self.best_obj):
                self.best_solution = cur_solution
                self.best_obj = cur_obj
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
            if (mark >= 3 or iter_num >= self.iter_num):
                stop = True
                print("algorithm stop")

    def runner(self):
        # run algorithm 
        self.main()

        return self.best_solution, self.best_obj

if __name__ == "__main__":
    # create instance
    w_num = 6
    l_num = 4
    bins_num = 18
    robot_num = 9
    picking_station_num = 6
    orders_num = 9
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    # create algorithm
    TRA_iter_num = 5000
    TRA_accept_gap = 0.2
    # each variable iter num
    x_iter_num = 10000
    y_iter_num = 10000
    z_iter_num = 100
    TRA_iter_num_dict = {
        "x": x_iter_num,
        "y": y_iter_num,
        "z": z_iter_num,
    }
    # each variable non improve count
    x_non_improve_count = 10000
    y_non_improve_count = 10000
    z_non_improve_count = 10000
    TRA_non_improve_count_dict = {
        "x": x_non_improve_count,
        "y": y_non_improve_count,
        "z": z_non_improve_count,
    }
    # each variable operator list
    x_operators_list = [operators_for_x.Relocate(instance=instance, k=1)]
    y_operators_list = []
    z_operators_list = [operators_for_z.Relocate(instance=instance, k=1)]
    TRA_operators_dict = {
        "x": x_operators_list,
        "y": y_operators_list,
        "z": z_operators_list,
    }
    # create TRA algorithm
    TRA_algorithm = TRAAlgorithmFramework(problem=instance, 
                                          iter_num=TRA_iter_num,
                                          accept_gap=TRA_accept_gap,
                                          iter_num_dict=TRA_iter_num_dict,
                                          non_improve_count_dict=TRA_non_improve_count_dict,
                                          operators_list_dict=TRA_operators_dict           
    )
    # run TRA algorithm
    solution, obj = TRA_algorithm.runner()




