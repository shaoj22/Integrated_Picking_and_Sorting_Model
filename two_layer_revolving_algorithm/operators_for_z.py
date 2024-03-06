'''
File: operators_for_z.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
as the file name, here are some operators for the z variable.
----------
Author: 626
Created Date: 2024.01.02
'''


import sys
sys.path.append('..')
from itertools import permutations
from generate_instances.Integrated_Instance import Instance


class Relocate():
    def __init__(self, instance=None, k=1):
        """ choose some orders and relocate its station idx """
        self.instance = instance
        self.k = k # how many orders relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, sorting_solution):
        """ relocate orders station idxs

        Args:
            sorting_solution (List[]): the solution code of the sorting.

        Returns:
            neighborhood (List[int]): idxs of the station of the variable zop of each neighborhood 
        """
        solution = sorting_solution.copy()
        neighborhood = []
        # 1. choose a order to relocate
        for o in range(len(solution)):
            # 2. choose a station to relocate
            for p in range(self.instance.P):
                neighbor = solution.copy()
                neighbor[o] = p
                if neighbor not in neighborhood:
                    neighborhood.append(neighbor)

        return neighborhood     

class Exchange():
    def __init__(self, k=1):
        self.k = k # how many points exchange together

    def run(self, solution):
        """exchange two points randomly inter/inner route (capacity not considered)
        ps: Exchange operator won't change the points number of each vehicle

        Args:
            solution (List[int]): idxs of points of each route (route seperate with idx 0)

        Returns:
            neighbours (List[List[int]]): idxs of points of each route (seperate with idx 0) of each neighbour 
        """
        neighbours = []
        # 1. choose point i
        for pi in range(1, len(solution)-2*self.k-1):
            # 2. choose point j
            for pj in range(pi+self.k+1, len(solution)-self.k): 
                # if math.prod(solution[pi:pi+self.k]) == 0 or math.prod(solution[pj:pj+self.k]) == 0: # don't exchange 0
                #     continue
                neighbour = solution.copy()
                tmp = neighbour[pi:pi+self.k].copy()
                neighbour[pi:pi+self.k] = neighbour[pj:pj+self.k]
                neighbour[pj:pj+self.k] = tmp
                neighbours.append(neighbour)
        return neighbours
    

if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 15
    robot_num = 10
    picking_station_num = 5
    orders_num = 5
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)


    solution = [0,1,2,3,4,5]
    operator = Relocate(instance=problem, k=1)
    neighborhood = operator.run(solution)
    print(neighborhood)