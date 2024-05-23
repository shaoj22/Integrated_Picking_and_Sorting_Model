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


import Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.operators_for_x
import Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.operators_for_y
import Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.operators_for_z
import Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.operators_for_u
# from TRA_utils import *
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.vns_framework_for_optimize_x import VNS as vns_for_optimize_x
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.vns_framework_for_optimize_y import VNS as vns_for_optimize_y
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.vns_framework_for_optimize_z import VNS as vns_for_optimize_z
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.vns_framework_for_optimize_u import VNS as vns_for_optimize_u
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.alns_framework_for_optimize_x1 import ALNS as alns_for_optimize_x1
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.initialization_for_TRA import initialization_for_TRA
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
import Integrated_Picking_and_Sorting_Model.utils_new as utils_new
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.operators_for_x1 import *
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm


class TRAAlgorithmFramework:
    def __init__(self, args=None):
        # input parameters
        self.problem = args['instance']
        self.algorithm_info = args['algorithm_info']

        # TRA parameters
        self.accept_gap = args['TRA_accept_gap']
        self.need_to_optimize_variable = args['need_to_optimize_variable']
        self.iter_num = args['TRA_iter_num']
        
        # Iter parameters
        self.iter_num_dict = args['TRA_iter_num_dict']
        self.x_iter_num = self.iter_num_dict['x']
        self.y_iter_num = self.iter_num_dict['y']
        self.z_iter_num = self.iter_num_dict['z']
        self.u_iter_num = self.iter_num_dict['u']
        self.x1_iter_num = self.iter_num_dict['x1']
        self.non_improve_count_dict = args['TRA_non_improve_count_dict']
        self.x_non_improve_count = self.non_improve_count_dict['x']
        self.y_non_improve_count = self.non_improve_count_dict['y']
        self.z_non_improve_count = self.non_improve_count_dict['z']
        self.u_non_improve_count = self.non_improve_count_dict['u']

        # operators list 
        self.operators_dict = args['TRA_operators_dict']
        self.x_operators_list = self.operators_dict['x']
        self.y_operators_list = self.operators_dict['y'] 
        self.z_operators_list = self.operators_dict['z']
        self.u_operators_list = self.operators_dict['u']

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
        
        # 优化x1
        if cur_optimize_variable == 'x1':
            x1_alns = alns_for_optimize_x1(instance=self.problem,
                                          iter_num=self.x1_iter_num,
                                          picking_solution=input_solution['picking_solution'],
                                          sorting_solution=input_solution['sorting_solution'],
                                          algorithm_info=self.algorithm_info
                                          )

            new_picking_solution, new_obj, self.algorithm_info = x1_alns.run()
            output_solution['picking_solution'] = new_picking_solution['picking']

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
        
        # 优化u
        elif cur_optimize_variable == 'u':
            u_vns = vns_for_optimize_u(problem=self.problem, 
                                       picking_solution=input_solution['picking_solution'],
                                       sorting_solution=input_solution['sorting_solution'],
                                       init_obj=input_obj,
                                       iter_num=self.u_iter_num,
                                       non_improve_count=self.u_non_improve_count,
                                       operators_list=self.u_operators_list
                                       )
            new_picking_solution, new_obj = u_vns.run()
            output_solution['picking_solution'] = new_picking_solution

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
            if ((self.best_obj - lower_bound_of_model)/self.best_obj - self.accept_gap) >= 0:
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
    w_num = 10
    l_num = 8
    bins_num = 60
    robot_num = 20
    picking_station_num = 10
    orders_num = 50
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    # create algorithm
    TRA_iter_num = 500
    TRA_accept_gap = 0.2
    # each variable iter num
    x_iter_num = 10000
    y_iter_num = 1000
    z_iter_num = 200
    u_iter_num = 10000
    x1_iter_num = 10000
    TRA_iter_num_dict = {
        "x": x_iter_num,
        "y": y_iter_num,
        "z": z_iter_num,
        "u": u_iter_num,
        "x1": x1_iter_num,
    }
    # each variable non improve count
    x_non_improve_count = 10000
    y_non_improve_count = 10000
    z_non_improve_count = 10000
    u_non_improve_count = 10000
    TRA_non_improve_count_dict = {
        "x": x_non_improve_count,
        "y": y_non_improve_count,
        "z": z_non_improve_count,
        "u": z_non_improve_count,
    }
    # each variable operator list
    # x_operators_list = [operators_for_x.relocateInner1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.relocateInner2(instance=instance, k=2)]
    # x_operators_list = [operators_for_x.relocateInter1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.exchangeInner1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.exchangeInter1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.reverseInner(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.reverseInter(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.greedyRelocateInner1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.greedyRelocateInter1(instance=instance, k=1)]
    # x_operators_list = [operators_for_x.greedyExchangeInter(instance=instance)]


    x_operators_list = [
        operators_for_x.relocateInner1(instance=instance, k=1),
        operators_for_x.relocateInter1(instance=instance, k=1),
        operators_for_x.exchangeInner1(instance=instance, k=1),
        operators_for_x.exchangeInter1(instance=instance, k=1),
        operators_for_x.reverseInner(instance=instance, k=1),
        operators_for_x.reverseInter(instance=instance, k=1),
        # operators_for_x.greedyRelocateInter1(instance=instance, k=1),
        # operators_for_x.greedyExchangeInter(instance=instance),

        ]
    
    y_operators_list = []

    z_operators_list = [operators_for_z.Relocate(instance=instance, k=1)]

    u_operators_list = [
        operators_for_u.relocateInter1(instance=instance, k=1),
        operators_for_u.exchangeInter1(instance=instance, k=1),
    ]

    TRA_operators_dict = {
        "x": x_operators_list,
        "y": y_operators_list,
        "z": z_operators_list,
        "u": u_operators_list,
    }

    break_operators_list = [
            PickingRandomBreak(instance), # base
            PickingRandomBreak(instance, break_num=2), # base
            PickingGreedyBreak(instance), # base 
            # PickingShawBreak(instance, break_num=2),

            SortingRandomBreak(instance), # base
            # SortingRandomBreak(instance, break_num=2),
            # SortingBalanceBreak(instance, break_num=2),
        ]
    repair_operators_list = [
            PickingRandomRepair(instance), # base
            PickingGreedyRepair(instance), # base
            SortingRandomRepair(instance), # base
            # SortingGreedyRepair(instance),
        ]
    
    break_operators_scores = np.ones(len(break_operators_list))
    repair_operators_scores = np.ones(len(repair_operators_list))
    break_operators_steps = np.ones(len(break_operators_list))
    repair_operators_steps = np.ones(len(repair_operators_list))

    tool = NNH_heuristic_algorithm(instance)
    picking_solution = tool.NNH_main()
    sorting_solution = [np.random.randint(instance.P) for _ in range(instance.O)]

    cur_solution = {
        "picking" : picking_solution,
        "sorting" : sorting_solution
    }

    algorithm_info = {
        "temperature" : 0.01,
        "break_operators_scores" : break_operators_scores,
        "repair_operators_scores" : repair_operators_scores, 
        "break_operators_steps" : break_operators_steps, 
        "repair_operators_steps" : repair_operators_steps, 
        "best_solution" : None,
        "best_obj" : 10000,
        "cur_solution" : cur_solution,
        "cur_obj" : None,
    }
    # create TRA algorithm
    TRA_algorithm = TRAAlgorithmFramework(problem=instance, 
                                          iter_num=TRA_iter_num,
                                          accept_gap=TRA_accept_gap,
                                          iter_num_dict=TRA_iter_num_dict,
                                          non_improve_count_dict=TRA_non_improve_count_dict,
                                          operators_list_dict=TRA_operators_dict,
                                          algorithm_info=algorithm_info,           
    )
    # run TRA algorithm
    solution, obj = TRA_algorithm.runner()
    print(solution['picking_solution'])
    # obj, info = utils_new.efficient_integrated_evaluate(instance, solution['picking_solution'], solution['sorting_solution'])
    # print(obj)




