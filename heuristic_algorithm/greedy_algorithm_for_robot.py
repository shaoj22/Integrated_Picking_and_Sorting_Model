'''
File: greedy_algorithm_for_robot.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a greedy algorithm for the init solution of the robot.
----------
Author: 626
Created Date: 2024.03.19
'''


import sys
sys.path.append('..')
from generate_instances.Integrated_Instance import Instance


class greedyAlgorithmForRobot():
    def __init__(self, instance):
        self.n = instance.n
        self.R = instance.robotNum
    
    def main(self):
        """ input instance, use greedy algorithm to generate the init solution for the robot. """
        routes = [[] for _ in range(self.R)]
        
        return routes

    def runner(self):
        """ runner of the rule based algorithm for robot. """
        routes = self.main()

        return routes


if __name__ == "__main__":
    w_num = 8
    l_num = 8
    bins_num = 10
    robot_num = 3
    picking_station_num = 5
    orders_num = 3
    instance = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    algorithm_tools = greedyAlgorithmForRobot(instance)
    routes = algorithm_tools.runner()
    print(routes)