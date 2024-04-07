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
import copy
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


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
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
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

        return neighborhood
  
  
class Exchange1():
    def __init__(self, instance=None, k=1):
        self.instance = instance
        self.k = k # how many points exchange together

    def run(self, picking_solution):
        """ exchange two p and d pairs randomly inner route
        ps: Exchange operator won't change the points number of each vehicle

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        # each robot
        robot_num = len(routes)
        for r in range(robot_num):
            # exchange num
            for e in range(self.k):
                # chose need to exchange's p
                p_list = [p for p in routes[r] if self.instance.node2type[p] in ["P1", "P2"]]
                if len(p_list) < 2:
                    continue
                # 遍历交换
                for p_1 in range(len(p_list)):
                    for p_2 in range(p_1+1, len(p_list)):
                        # need exchange's p1 p2 d1 d2
                        p_choose_1 = p_list[p_1]
                        p_choose_2 = p_list[p_2]
                        d_choose_1 = p_choose_1 + 2*self.instance.n
                        d_choose_2 = p_choose_2 + 2*self.instance.n
                        # 计算p1和p2和d1和d2在route[r]中的idx
                        p_idx_1 = routes[r].index(p_choose_1)
                        p_idx_2 = routes[r].index(p_choose_2)
                        d_idx_1 = routes[r].index(d_choose_1)
                        d_idx_2 = routes[r].index(d_choose_2)
                        # 交换加入neighborhood
                        neighbor = copy.deepcopy(routes)
                        temp_p = neighbor[r][p_idx_1]
                        temp_d = neighbor[r][d_idx_1]
                        neighbor[r][p_idx_1] = neighbor[r][p_idx_2]
                        neighbor[r][p_idx_2] = temp_p
                        neighbor[r][d_idx_1] = neighbor[r][d_idx_2]
                        neighbor[r][d_idx_2] = temp_d
                        if neighbor not in neighborhood:
                            neighborhood.append(neighbor)
        
        return neighborhood


class Exchange2():
    def __init__(self, instance=None, k=1):
        self.instance = instance
        self.k = k # how many points exchange together

    def run(self, picking_solution):
        """ exchange two p and d pairs randomly inter route
        ps: Exchange operator won't change the points number of each vehicle

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        cur_choose_num = 0
        while cur_choose_num < self.k:
            # random get tow robot
            r_1 = np.random.randint(0, len(routes))
            r_2 = np.random.randint(0, len(routes))
            if r_1 == r_2:
                continue
            p_1_list = [p for p in routes[r_1] if self.instance.node2type[p] in ["P1", "P2"]]
            p_2_list = [p for p in routes[r_2] if self.instance.node2type[p] in ["P1", "P2"]]
            # 如果p1或者p2中有空的则continue
            if len(p_1_list) == 0 or len(p_2_list) == 0:
                continue
            # 遍历交换p1个p2
            for p_1 in range(len(p_1_list)):
                for p_2 in range(len(p_2_list)):
                    # need exchange's p1 p2 d1 d2
                    p_choose_1 = p_1_list[p_1]
                    p_choose_2 = p_2_list[p_2]
                    d_choose_1 = p_choose_1 + 2*self.instance.n
                    d_choose_2 = p_choose_2 + 2*self.instance.n
                    # 计算p1和p2和d1和d2在route[r]中的idx
                    p_idx_1 = routes[r_1].index(p_choose_1)
                    p_idx_2 = routes[r_2].index(p_choose_2)
                    d_idx_1 = routes[r_1].index(d_choose_1)
                    d_idx_2 = routes[r_2].index(d_choose_2)
                    # 交换加入neighborhood
                    neighbor = copy.deepcopy(routes)
                    temp_p = neighbor[r_1][p_idx_1]
                    temp_d = neighbor[r_1][d_idx_1]
                    neighbor[r_1][p_idx_1] = neighbor[r_2][p_idx_2]
                    neighbor[r_2][p_idx_2] = temp_p
                    neighbor[r_1][d_idx_1] = neighbor[r_2][d_idx_2]
                    neighbor[r_2][d_idx_2] = temp_d
                    if neighbor not in neighborhood:
                        neighborhood.append(neighbor)
            cur_choose_num += 1
            
        return neighborhood


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
    

class newRelocate():
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
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        # transfer routes to solution: list of all robot's route
        solution = transfer_picking_solution_2_list(routes)
        # 1. choose a point to relocate
        for pi in range(1, len(solution)-self.k):
            # 2. choose a position to put
            for li in range(1, len(solution)-self.k): # can't relocate to start/end
                neighbour = solution.copy()
                points = []
                for _ in range(self.k):
                    points.append(neighbour.pop(pi))
                for p in points[::-1]:
                    neighbour.insert(li, p)
                neighborhood.append(neighbour)
        # transfer back to routes
        for n in range(len(neighborhood)):
            neighborhood[n] = transfer_list_2_picking_solution(neighborhood[n], self.instance.n)
        
        return neighborhood


def transfer_picking_solution_2_list(picking_solution):
    """ transfer picking solution to list of list of points

    Args:
        picking_solution (List[List[int]]): idxs of points of each route

    Returns:
        solution (List[int]): idxs of points of each route (separate with idx -1)
    """
    solution = []
    for r in range(len(picking_solution)):
        for p in range(len(picking_solution[r])):
            if p == 0:
                solution.append(-1) # separate with idx -1
            else:
                solution.append(picking_solution[r][p])
    solution.append(-1)
    
    return solution


def transfer_list_2_picking_solution(solution, node_num):
    """ transfer list of points to picking solution
    
    Args:
        solution (List[int]): idxs of points of each route (separate with idx -1)
        node_num (int): number of nodes

    Returns:    
        picking_solution (List[List[int]]): idxs of points of each route
    """
    picking_solution = []
    r = -1
    for p in range(len(solution)):
        if solution[p] == -1:
            if p+1 == len(solution):
                break
            # create a new robot
            r += 1
            picking_solution.append([])
            # add robot's start point
            picking_solution[r].append(node_num*4+r)
        else:
            # add robot's points
            picking_solution[r].append(solution[p])

    return picking_solution