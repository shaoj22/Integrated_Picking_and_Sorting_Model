'''
File: greedy_algorithm_for_order.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a greedy algorithm for the init solution of the order.
----------
Author: 626
Created Date: 2024.03.19
'''


import sys
sys.path.append('..')
import random
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


class greedyAlgorithmForOrder():
    def __init__(self, instance: Instance) -> None:
        self.O = instance.O
        self.P = instance.P
        self.N = instance.n
        self.IO = instance.IO
    
    def main(self):
        """ input instance, use greedy algorithm to generate the init solution for the order. """
        assignment_solution = [0 for _ in range(self.O)]
        picking_station_buffer = [[] for _ in range(self.P)] # each picking station's order list.
        order_list = [o for o in range(self.O)]
        # init add one order to the each picking station.
        for p in range(self.P):
            if len(order_list) == 0:
                break
            order = random.choice(order_list)
            picking_station_buffer[p].append(order)
            assignment_solution[order] = p
            order_list.remove(order)
        # order finished.
        if len(order_list) == 0:
            return assignment_solution
        # traverse the picking station, choose the greedy order to add.
        picking_station_sku = [[] for _ in range(self.P)]
        # add init sku.
        for p in range(self.P):
            for sku in range(self.N):
                order = picking_station_buffer[p][0]
                if self.IO[sku][order] == 1:
                    picking_station_sku[p].append(sku)
        # choose next order into picking station.
        order_finished = False
        p = 0
        while not order_finished:
            # base case 
            if len(order_list) == 0:
                order_finished = True
                break
            chosen_order_idx = 0
            sku_max_match_num = 0
            # traverse the order list.
            for o in range(len(order_list)):
                order = order_list[o]
                sku_match_num = 0
                # traverse the sku list and cal the sku match num.
                for sku in range(self.N):
                    if self.IO[sku][order] == 1 and sku in picking_station_sku[p]:
                        sku_match_num += 1
                if sku_match_num > sku_max_match_num:
                    sku_max_match_num = sku_match_num
                    chosen_order_idx = o
            # add the chosen order into the picking station.
            picking_station_buffer.append(order_list[chosen_order_idx])
            assignment_solution[order_list[chosen_order_idx]] = p
            # add the chosen order's sku into the picking station.
            for sku in range(self.N):
                if self.IO[sku][order_list[chosen_order_idx]] == 1:
                    picking_station_sku[p].append(sku)
            order_list.remove(order_list[chosen_order_idx])
            # next picking station.
            if p == self.P - 1:
                p = 0
            else:
                p += 1
            
        return assignment_solution

    def runner(self):
        """ runner of the greedy algorithm for order. """
        assignment_solution = self.main()

        return assignment_solution
    

if __name__ == "__main__":
    w_num = 8
    l_num = 8
    bins_num = 100
    robot_num = 3
    picking_station_num = 5
    orders_num = 100
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    algorithm_tools = greedyAlgorithmForOrder(instance)
    assignment_solution = algorithm_tools.runner()
    print(assignment_solution)