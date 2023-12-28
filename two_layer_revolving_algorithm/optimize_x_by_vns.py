'''
File: optimize_x_by_vns.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
use vns framework to optimize the variable x.
----------
Author: 626
Created Date: 2023.12.27
'''

from vns_framework_for_optimize_variable import VNS



def optimize_x_by_vns(problem=None, input_variable=None, iter_num=1000, non_improve_count=100, operators_list=None):
    """ use vns to optimize the x variable.

    Args:
        problem (class): the instance of the input problem.
        input_variable (class): variable optimized.
        iter_num (int): iter num for the vns to run.
        non_improve_count (int): more than this count, the process will be closed.
        operators_list ([]): list include the operators for optimize the variable.

    Returns:
        best_variable (class): the best variable after this optimize by vns.
    """
    x_vns = VNS(problem, input_variable, iter_num, non_improve_count, operators_list)
    x_vns.run()
    best_variable = x_vns.best_variable
    
    return best_variable

        

