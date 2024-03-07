'''
File: initialization_for_TRA.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
use the ALNS to get the init solution of input problem and variable
----------
Author: 626
Created Date: 2023.12.29
'''


import sys
sys.path.append('..')
import numpy as np
import utils
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm as picking_greedy_alg


def initialization_for_TRA(problem):
    """ input the problem and variable and then use the ALNS algorithm to generate the init solution for the variable 
    
    Args:
        problem (class): instance of the problem.
        variable ({}): variable of the model.

    Return:
        init_variable ({}): after optimized by greedy algorithm or ALNS algorithm.
        solution ({}): after optimized by greedy algorithm or ALNS algorithm, include picking and sorting solution.
        init_obj (int): the init obj after optimized by greedy or ALNS algorithm. 
    """
    # create the picking greedy algorithm
    picking_greedy_algorithm = picking_greedy_alg(instance=problem)
    picking_solution = picking_greedy_algorithm.NNH_main()
    # create the sorting greedy algorithm
    sorting_solution = [np.random.randint(problem.P) for _ in range(problem.O)]
    # record the init solution of picking and sorting
    solution = {
        'picking_solution': picking_solution,
        'sorting_solution': sorting_solution
    }
    # record the obj
    obj, info = utils.efficient_integrated_evaluate(problem, picking_solution, sorting_solution)
    
    return solution, obj