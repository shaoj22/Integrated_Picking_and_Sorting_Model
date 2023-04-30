'''
File: Picking_VNS.py
Project: Integrated_Picking---Sorting_Model
File Created: Wednesday, 26th April 2023 7:22:16 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''

import numpy as np
import matplotlib.pyplot as plt
from tqdm import trange
import time
from Picking_Instance import Instance
from Picking_Operators import *
import utils

class Picking_VNS:
    def __init__(self, picking_instance, iter_num):
        self.picking_instance = picking_instance
        self.iter_num = iter_num

        self.choose_neighbour_strategy = "last"
        # set VNS paraments
        self.operators_list = [RelocateD(picking_instance), RellocatePD(picking_instance)]
        self.process = []

    def solution_init(self):
        """
        generate feasible initial solution
        """
        solution = []
        # create routes with W
        for k in self.picking_instance.K:
            w = self.picking_instance.W[k] # depot of vehicle k
            route = [w]
            solution.append(route)
        # fill routes with P1, D1
        for i in range(self.picking_instance.n):
            p1 = self.picking_instance.P1[i]
            d1 = self.picking_instance.D1[i]
            k = i % self.picking_instance.robotNum
            solution[k].append(p1)
            solution[k].append(d1)
        # fill routes with P2, D2
        for i in range(self.picking_instance.n):
            p2 = self.picking_instance.P2[i]
            d2 = self.picking_instance.D2[i]
            k = i % self.picking_instance.robotNum
            solution[k].append(p2)
            solution[k].append(d2)
        return solution

    def transfer(self, solution):
        x = np.zeros((self.picking_instance.nodeNum, self.picking_instance.nodeNum))
        for k in range(self.picking_instance.robotNum):
            route = solution[k]
            for i in range(1, len(route)):
                x[route[i-1], route[i]] = 1
            x[route[-1], route[0]] = 1
        return x

    def cal_objective(self, solution):
        return utils.efficient_picking_evaluate(self.picking_instance, solution)

    def cal_objective_with_model(self, solution):
        x_val = self.transfer(solution)
        obj = utils.picking_evaluate(self.picking_instance, x_val)
        return obj

    def get_neighbours(self, solution, operator):
        neighbours = operator.run(solution)
        return neighbours

    def choose_neighbour(self, neighbours):
        # randomly choose neighbour
        if self.choose_neighbour_strategy == "random":
            chosen_ni = np.random.randint(len(neighbours))
        # choose the first neighbour
        if self.choose_neighbour_strategy == "last":
            chosen_ni = -1
        # choose the best neighour
        elif self.choose_neighbour_strategy == "best":
            best_obj = np.inf
            for ni, neighbour in enumerate(neighbours):
                obj = self.cal_objective(neighbour) 
                if obj < best_obj:
                    best_obj = obj
                    best_ni = ni
            chosen_ni = best_ni
        return chosen_ni
    
    def show_process(self):
        y = self.process
        x = np.arange(len(y))
        plt.plot(x, y)
        plt.show()
   
    def run(self):
        best_solution = self.solution_init() # solution in form of routes
        best_obj = self.cal_objective(best_solution)
        neighbours = self.get_neighbours(best_solution, operator=self.operators_list[0])
        operator_k = 0
        for step in trange(self.iter_num):
            ni = self.choose_neighbour(neighbours)
            cur_solution = neighbours[ni]
            cur_obj = self.cal_objective(cur_solution)
            # obj: minimize the total distance 
            if cur_obj < best_obj: 
                # self.operators_list.insert(0, self.operators_list.pop(operator_k))
                operator_k = 0
                best_solution = cur_solution
                best_obj = cur_obj
                neighbours = self.get_neighbours(best_solution, operator=self.operators_list[0])
            else:
                neighbours.pop(ni)
                if len(neighbours) == 0: # when the neighbour space empty, change anothor neighbour structure(operator)
                    operator_k += 1
                    if operator_k < len(self.operators_list):
                        operator = self.operators_list[operator_k]
                        neighbours = self.get_neighbours(best_solution, operator=operator)
                    else:
                        print('local optimal, break out, iterated {} times'.format(step))
                        break

            self.process.append(best_obj)
        self.best_solution = best_solution
        self.best_obj = best_obj
        self.best_x_val = self.transfer(self.best_solution)
        return self.best_x_val, self.best_obj

if __name__ == "__main__":
    w_num = 2
    l_num = 2
    task_num = 10
    robot_num = 10
    picking_instance = Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_VNS(picking_instance, 100000)
    start = time.time()
    alg.run()
    end = time.time()
    print("evaluate_obj = {}".format(alg.cal_objective_with_model(alg.best_solution)))
    print("best_obj = {}, time_cost = {}, best_solution:\n".format(alg.best_obj, end-start), alg.best_solution)
    alg.show_process()
        
