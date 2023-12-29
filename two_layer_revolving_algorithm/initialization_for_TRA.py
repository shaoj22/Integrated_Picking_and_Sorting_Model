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


def initialization_for_TRA(problem, variable):
    """ input the problem and variable and then use the ALNS algorithm to generate the init solution for the variable 
    
    Args:
        problem (class): instance of the problem.
        variable (class): variable of the model.

    Return:
        init_variable (class): after optimized by ALNS algorithm.
    """

    init_variable = variable
    


    return init_variable