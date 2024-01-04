'''
File: common_algorithm_by_strengthened_gurobi.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a algorithm to get the objective of the model, after fixed x\y\z variable, and some variable that can be cal by x\y\z.
----------
Author: 626
Created Date: 2024.01.04
'''


import sys
sys.path.append('..')
import numpy as np
import gurobipy as gp
import time 
import utils
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from generate_instances.Integrated_Instance import Instance
from two_layer_revolving_algorithm.integrated_gurobi_model_update import IntegratedGurobiModel
from two_layer_revolving_algorithm.TRA_utils import *
from gurobipy import GRB
from two_layer_revolving_algorithm.Variable import Variable


class commonAlgorithmByGurobi:
    pass



if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 100
    robot_num = 20
    picking_station_num = 5
    orders_num = 40
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    # 获取初始解
    input_variable = Variable(problem)
    picking_alg = NNH_heuristic_algorithm(problem)
    picking_solution = picking_alg.NNH_main()
    sorting_solution = [np.random.randint(problem.P) for _ in range(problem.O)]
    x_val, y_val, z_val = utils.solution_transfer(problem, picking_solution, sorting_solution)
    variable_dict = {
            'x' : x_val,
            'y' : y_val,
            'z' : z_val
        }
    input_variable.set_x_variable(variable_dict)
    input_variable.set_y_variable(variable_dict)
    input_variable.set_z_variable(variable_dict)
    solver2 = commonAlgorithmByGurobi(problem, input_variable)
    time3 = time.time()
    model2 = solver2.run_gurobi_model()
    time4 = time.time()
    print(time4-time3)