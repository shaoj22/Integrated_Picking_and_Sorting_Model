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
from common_algorithm_by_gurobi import commonAlgorithmByGurobi
from TRA_utils import *

class VNS:
    def __init__(self, input_variable=None, iter_num=1000, operators_list=None):
        """ input model's variable and use vns framework to optimize the variable.

        Args:
            input_variable (class): variable optimized.
            iter_num (int): iter num for the vns to run.
            operators_list ([]): list include the operators for optimize the variable.

        Returns:
            best_variable (class): the best variable after this optimize by vns.
        """

        # input parameters
        self.input_variable = input_variable
        self.iter_num = iter_num
        self.operators_list = operators_list

    def solution_init(self):
        """ use the current variable and the common algorithm to get init solution for the problem """
        pass
                
    def cal_objective_and_variable(self, problem, variable):
        """ calculate the objective of the variable 
        
        Args:
            problem (class): instance;
            variable (class): x\y\z variable.
        
        Returns:
            variable_update (class): the update variable after solving by common algorithm;
            objective_value (double): the objective of the problem with the variable.
        """
        solver = commonAlgorithmByGurobi(problem, variable) # init the solver
        model = solver.run_gurobi_model() # run the solver
        variable_update = get_variable_from_solved_model() # get the update variable
        objective_value = get_objective_from_solved_model(model) # get the update objective value

        return variable_update, objective_value

    def get_neighborhood(self, solution, operator):
        neighborhood = operator.run(solution)
        return neighborhood
    
    def choose_neighborhood(self, neighborhood):
        chosen_ni = np.random.randint(len(neighborhood))
        return chosen_ni

    def run(self):
        self.solution_init() # solution in form of routes
        neighbours = self.get_neighbours(self.best_solution, operator=self.operators_list[0])
        operator_k = 0
        for step in trange(self.iter_num):
            ni = self.choose_neighbour(neighbours)
            cur_solution = neighbours[ni]
            cur_solution = self.remove_empty_vehicle(cur_solution)
            cur_obj = self.cal_objective(cur_solution)
            # obj: minimize the total distance 
            if cur_obj < self.best_obj: 
                self.operators_list.insert(0, self.operators_list.pop(operator_k))
                operator_k = 0
                self.best_solution = cur_solution
                self.best_obj = cur_obj
                neighbours = self.get_neighbours(self.best_solution, operator=self.operators_list[0])
            else:
                neighbours.pop(ni)
                if len(neighbours) == 0: # when the neighbour space empty, change anothor neighbour structure(operator)
                    operator_k += 1
                    if operator_k < len(self.operators_list):
                        operator = self.operators_list[operator_k]
                        neighbours = self.get_neighbours(self.best_solution, operator=operator)
                    else:
                        print('local optimal, break out, iterated {} times'.format(step))
                        break

            self.process.append(self.best_obj)
        self.best_routes = self.transfer(self.best_solution)
        return self.best_routes   


if __name__ == "__main__":
    input_path = "D:\\Desktop\\python_code\\China_Mobile_City_Delivery\\inputs\\myInstance4"
    graph = GraphTools.Graph(input_path=input_path)    
    alg = VNS(graph, iter_num=100000, heuristic=None)
    routes = alg.run()
    print(routes)
    # alg.show_result() 
    # alg.show_process()
    # alg.show_routes()
    # output_result(graph, routes)


    
                
                




