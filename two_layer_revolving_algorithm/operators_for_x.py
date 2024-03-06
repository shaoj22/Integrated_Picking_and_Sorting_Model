'''
File: operators_for_x.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
as the file name, here are some operators for the x variable.
----------
Author: 626
Created Date: 2024.01.02
'''

import sys
sys.path.append('..')
import numpy as np
import math
import copy
from generate_instances.Integrated_Instance import Instance


class Relocate():
    def __init__(self, instance=None, k=1):
        """ choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inter/inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        # routes = copy.deepcopy(picking_solution)
        routes = picking_solution
        neighborhood = [] # 邻域集合
        # neighborhood = set() # 邻域集合
        choose_p_list = [] # 当前选择的p
        choose_d_list = [] # 当前选择对应的d
        cur_choose_num = 0 # 当前的num
        while cur_choose_num < self.k:
            # random choose one robot
            k = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p_list = [p for p in routes[k] if self.instance.node2type[p] in ["P1", "P2"]]
            if len(p_list) == 0:
                continue
            # 获取起终点
            p_choose = np.random.choice(p_list)
            d_choose = p_choose + 2*self.instance.n
            # 删除起终点
            routes[k].remove(p_choose)
            routes[k].remove(d_choose)
            # 保留删除的点
            choose_p_list.append(p_choose)
            choose_d_list.append(d_choose)
            cur_choose_num += 1
        # 根据chosen的pd来获取neighborhood
        for i in range(self.k): # 选择第几个p d pair
            p_insert = choose_p_list[i] # 需要插入的p
            d_insert = choose_d_list[i] # 需要插入的d
            route_num = len(routes)
            for r in range(route_num): # 选择第几个robot插入
                """ 
                tips: pos_p < pos_d
                """
                route_point_num = len(routes[r])
                for pos_p in range(1, route_point_num): # 不能插入第一个点
                    for pos_d in range(pos_p, route_point_num): # d在p之后
                        neighbor = copy.deepcopy(routes)
                        # 插入p和d
                        neighbor[r].insert(pos_p, p_insert)
                        neighbor[r].insert(pos_d, d_insert)
                        if neighbor not in neighborhood:
                            neighborhood.append(neighbor)

                        # neighbor = copy.deepcopy(routes)  # 在这里进行深拷贝
                        # # 插入p和d
                        # neighbor[r].insert(pos_p, p_insert)
                        # neighbor[r].insert(pos_d, d_insert)
                        # if tuple(map(list, neighbor)) not in neighborhood:  # 检查是否已存在于集合中
                        #     neighborhood.add(tuple(map(list, neighbor)))

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

    def get(self, solution):
        pi = np.random.randint(1, len(solution)-2*self.k-1)
        pj = np.random.randint(pi+self.k+1, len(solution)-self.k)
        while math.prod(solution[pi:pi+self.k]) == 0 or math.prod(solution[pj:pj+self.k]) == 0: # don't exchange 0
            pi = np.random.randint(1, len(solution)-2*self.k-1)
            pj = np.random.randint(pi+self.k+1, len(solution)-self.k)
        neighbour = solution.copy()
        tmp = neighbour[pi:pi+self.k].copy()
        neighbour[pi:pi+self.k] = neighbour[pj:pj+self.k]
        neighbour[pj:pj+self.k] = tmp
        assert len(neighbour) == len(solution)
        return neighbour

class Reverse():
    def __init__(self):
        pass

    def run(self, solution):
        """reverse route between two points randomly inter/inner route (capacity not considered)

        Args:
            solution (List[int]): idxs of points of each route (route seperate with idx 0)

        Returns:
            neighbours (List[List[int]]): idxs of points of each route (seperate with idx 0) of each neighbour 
        """
        neighbours = []
        # 1. choose point i
        for pi in range(1, len(solution)-2):
            # 2. choose point j
            for pj in range(pi+1, len(solution)-1): 
                neighbour = solution.copy()
                neighbour[pi:pj+1] = neighbour[pj:pi-1:-1]
                neighbours.append(neighbour)
        return neighbours 
    
    def get(self, solution):
        pi = np.random.randint(1, len(solution)-2)
        pj = np.random.randint(pi+1, len(solution)-1)
        neighbour = solution.copy()
        neighbour[pi:pj+1] = neighbour[pj:pi-1:-1]
        assert len(neighbour) == len(solution)
        return neighbour