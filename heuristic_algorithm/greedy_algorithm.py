'''
File: greedy_algorithm.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a greedy algorithm for the integrated model.
----------
Author: 626
Created Date: 2024.03.20
'''


import sys
sys.path.append('..')
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.greedy_algorithm_for_order import greedyAlgorithmForOrder
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.greedy_algorithm_for_robot import greedyAlgorithmForRobot


class greedyAlgorithm():
    def __init__(self, instance: Instance) -> None:
        self.instance = instance
    
    def main(self):
        """ input instance, use greedy algorithm to generate the solution for the integrated model. """
        order_algorithm_tools = greedyAlgorithmForOrder(self.instance)
        assignment_solution = order_algorithm_tools.runner()
        robot_algorithm_tools = greedyAlgorithmForRobot(self.instance)
        routes = robot_algorithm_tools.runner()
        solution = {
            "sorting_solution": assignment_solution,
            "picking_solution": routes,
        }

        return solution

    def runner(self):
        """ runner of the greedy algorithm. """
        solution = self.main()

        return solution
    

if __name__ == "__main__":
    w_num = 8
    l_num = 8
    bins_num = 100
    robot_num = 3
    picking_station_num = 5
    orders_num = 100
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    algorithm_tools = greedyAlgorithm(instance)
    solution = algorithm_tools.runner()
    print(solution)