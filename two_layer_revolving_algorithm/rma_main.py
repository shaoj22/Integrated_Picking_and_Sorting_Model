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


import numpy as np
import operators_for_x
import operators_for_y
import operators_for_z
import operators_for_u
import operators_for_x1
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.TRA_algorithm_framework import TRAAlgorithmFramework
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.greedy_algorithm_for_order import greedyAlgorithmForOrder
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.greedy_algorithm_for_robot import greedyAlgorithmForRobot


def rma_main(instance=None):
    """ create the rma algorithm and input some args to run main 
    
    
    
    """
    need_to_optimize_variable = ['x1', 'y', 'z']

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

    z_operators_list = [
        operators_for_z.Relocate(instance=instance, k=1)
    ]

    u_operators_list = [
        operators_for_u.relocateInter1(instance=instance, k=1),
        operators_for_u.exchangeInter1(instance=instance, k=1),
    ]

    break_operators_list = [
        operators_for_x1.PickingRandomBreak(instance), # base
        operators_for_x1.PickingRandomBreak(instance, break_num=2), # base
        operators_for_x1.PickingGreedyBreak(instance), # base 
        # operators_for_x1.PickingShawBreak(instance, break_num=2),
        operators_for_x1.SortingRandomBreak(instance), # base
        # operators_for_x1.SortingRandomBreak(instance, break_num=2),
        # operators_for_x1.SortingBalanceBreak(instance, break_num=2),
    ]

    repair_operators_list = [
        operators_for_x1.PickingRandomRepair(instance), # base
        operators_for_x1.PickingGreedyRepair(instance), # base
        operators_for_x1.SortingRandomRepair(instance), # base
        # SortingGreedyRepair(instance),
    ]

    # picking_alg = NNH_heuristic_algorithm(instance)
    # picking_solution = picking_alg.NNH_main()
    # sorting_solution = [np.random.randint(instance.P) for _ in range(instance.O)]
    
    order_algorithm_tools = greedyAlgorithmForOrder(instance)
    sorting_solution = order_algorithm_tools.runner()
    robot_algorithm_tools = greedyAlgorithmForRobot(instance)
    picking_solution = robot_algorithm_tools.runner()

    cur_solution = {
        "picking" : picking_solution,
        "sorting" : sorting_solution
    }

    break_operators_scores = np.ones(len(break_operators_list))
    repair_operators_scores = np.ones(len(repair_operators_list))
    break_operators_steps = np.ones(len(break_operators_list))
    repair_operators_steps = np.ones(len(repair_operators_list))

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
    
    args = {
        "instance" : instance,
        "TRA_iter_num" : 500,
        "TRA_accept_gap" : 0.2,
        "TRA_iter_num_dict" : {
            "x": 10000,
            "y": 1000,
            "z": 500,
            "u": 10000,
            "x1": 10000,
        },
        "TRA_non_improve_count_dict" : {
            "x": 10000,
            "y": 10000,
            "z": 10000,
            "u": 10000,
            "x1": 10000,
        },
        "TRA_operators_dict" : {
            "x": x_operators_list,
            "y": y_operators_list,
            "z": z_operators_list,
            "u": u_operators_list,
        },
        "break_operators_list" : break_operators_list,
        "repair_operators_list" : repair_operators_list,
        "algorithm_info" : algorithm_info,
        "need_to_optimize_variable" : need_to_optimize_variable,
    }

    TRA_algorithm = TRAAlgorithmFramework(args=args)

    solution, obj = TRA_algorithm.runner()

    return solution, obj


if __name__ == "__main__":
    # create instance
    w_num = 4
    l_num = 4
    bins_num = 12
    robot_num = 6
    picking_station_num = 5
    orders_num = 6
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    solution, obj = rma_main(instance)
