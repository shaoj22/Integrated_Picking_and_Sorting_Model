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


class relocateInner1():
    def __init__(self, instance=None, k=1):
        """ choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        p_list = []
        while len(p_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p_list = [p for p in routes[k] if self.instance.node2type[p] in ["P1", "P2"]]
        # 获取起终点
        p_choose = np.random.choice(p_list)
        d_choose = p_choose + 2*self.instance.n
        # 删除起终点
        routes[k].remove(p_choose)
        routes[k].remove(d_choose)
        # 根据chosen的pd来获取neighborhood
        route_point_num = len(routes[k])
        """ tips: pos_p < pos_d """
        for pos_p in range(1, route_point_num+1): # 不能插入第一个点
            for pos_d in range(pos_p, route_point_num+1): # d在p之后
                neighbor = copy.deepcopy(routes)
                # 插入p和d
                neighbor[k].insert(pos_p, p_choose)
                neighbor[k].insert(pos_d, d_choose)
                if neighbor not in neighborhood:
                    neighborhood.append(neighbor)

        return neighborhood


class relocateInner2():
    def __init__(self, instance=None, k=2):
        """ choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        p1_list = []
        p2_list = []
        while len(p1_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k1 = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p1_list = [p for p in routes[k1] if self.instance.node2type[p] in ["P1", "P2"]]
        while len(p2_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k2 = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p2_list = [p for p in routes[k2] if self.instance.node2type[p] in ["P1", "P2"]]
        p1_choose = np.random.choice(p1_list)
        p2_choose = np.random.choice(p2_list)
        d1_choose = p1_choose + 2*self.instance.n
        d2_choose = p2_choose + 2*self.instance.n
        # 删除起终点
        routes[k1].remove(p1_choose)
        routes[k1].remove(d1_choose)
        routes[k2].remove(p2_choose)
        routes[k2].remove(d2_choose)
        # 根据chosen的pd来获取neighborhood
        route_point_num1 = len(routes[k1])
        route_point_num2 = len(routes[k2])
        for pos_p1 in range(1, route_point_num1+1):
            for pos_d1 in range(pos_p1, route_point_num1+1):
                for pos_p2 in range(1, route_point_num2+1):
                    for pos_d2 in range(pos_p2, route_point_num2+1):
                        neighbor = copy.deepcopy(routes)
                        # 插入p和d
                        neighbor[k1].insert(pos_p1, p1_choose)
                        neighbor[k1].insert(pos_d1, d1_choose)
                        neighbor[k2].insert(pos_p2, p2_choose)
                        neighbor[k2].insert(pos_d2, d2_choose)
                        if neighbor not in neighborhood:
                            neighborhood.append(neighbor)

        return neighborhood


class relocateInter1():
    def __init__(self, instance=None, k=1):
        """ choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inter route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        p_list = []
        while len(p_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p_list = [p for p in routes[k] if self.instance.node2type[p] in ["P1", "P2"]]
        # 获取起终点
        p_choose = np.random.choice(p_list)
        d_choose = p_choose + 2*self.instance.n
        # 删除起终点
        routes[k].remove(p_choose)
        routes[k].remove(d_choose)
        # 根据chosen的pd来获取neighborhood
        route_num = len(routes)
        for r in range(route_num): # 选择第几个robot插入
            """ tips: pos_p < pos_d """
            route_point_num = len(routes[r])
            for pos_p in range(1, route_point_num): # 不能插入第一个点
                for pos_d in range(pos_p, route_point_num): # d在p之后
                    neighbor = copy.deepcopy(routes)
                    # 插入p和d
                    neighbor[r].insert(pos_p, p_choose)
                    neighbor[r].insert(pos_d, d_choose)
                    if neighbor not in neighborhood:
                        neighborhood.append(neighbor)

        return neighborhood


class exchangeInner1():
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


class exchangeInter1():
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
        # random get tow robot
        p1_list = []
        p2_list = []
        while len(p1_list) == 0 or len(p2_list) == 0:
            # random choose two robot
            """ TODO: relocate each robot rather than random two """
            r1, r2 = np.random.choice(range(len(routes)), 2, replace=False)
            # random choose one start point of this robot
            p1_list = [p for p in routes[r1] if self.instance.node2type[p] in ["P1", "P2"]]
            p2_list = [p for p in routes[r2] if self.instance.node2type[p] in ["P1", "P2"]]
        # 遍历交换p1个p2
        for p_1 in range(len(p1_list)):
            for p_2 in range(len(p2_list)):
                # need exchange's p1 p2 d1 d2
                p_choose_1 = p1_list[p_1]
                p_choose_2 = p2_list[p_2]
                d_choose_1 = p_choose_1 + 2*self.instance.n
                d_choose_2 = p_choose_2 + 2*self.instance.n
                # 计算p1和p2和d1和d2在route[r]中的idx
                p_idx_1 = routes[r1].index(p_choose_1)
                p_idx_2 = routes[r2].index(p_choose_2)
                d_idx_1 = routes[r1].index(d_choose_1)
                d_idx_2 = routes[r2].index(d_choose_2)
                # 交换加入neighborhood
                neighbor = copy.deepcopy(routes)
                temp_p = neighbor[r1][p_idx_1]
                temp_d = neighbor[r1][d_idx_1]
                neighbor[r1][p_idx_1] = neighbor[r2][p_idx_2]
                neighbor[r2][p_idx_2] = temp_p
                neighbor[r1][d_idx_1] = neighbor[r2][d_idx_2]
                neighbor[r2][d_idx_2] = temp_d
                if neighbor not in neighborhood:
                    neighborhood.append(neighbor)

        return neighborhood


class reverseInner():
    def __init__(self, instance=None, k=1): 
        self.instance = instance
        self.k = k # how many points reverse together

    def run(self, solution):
        """ reverse route between two points randomly inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(solution)
        neighborhood = [] # 邻域集合
        cur_choose_num = 0
        while cur_choose_num < self.k:
            # random choose one robot
            r = np.random.randint(0, len(routes))
            # get the p1 and p2 list
            p_list = [p for p in routes[r] if self.instance.node2type[p] in ["P1", "P2"]]
            if len(p_list) < 2:
                continue
            # reverse the p list
            for pi in range(0, len(p_list)-1):
                for pj in range(pi+1, len(p_list)): 
                    new_p_list = p_list.copy()
                    new_p_list[pi:pj+1] = new_p_list[pj:pi-1:-1]
                    # 根据new_p_list重新生成neighbor并加入neighborhood
                    neighbor = copy.deepcopy(routes)
                    for p in range(len(new_p_list)):
                        # need exchange's p1 p2 d1 d2
                        p_choose_1 = p_list[p]
                        p_choose_2 = new_p_list[p]
                        if p_choose_1 == p_choose_2:
                            continue
                        d_choose_1 = p_choose_1 + 2*self.instance.n
                        d_choose_2 = p_choose_2 + 2*self.instance.n
                        # 计算p1和p2和d1和d2在route[r]中的idx
                        p_idx_1 = routes[r].index(p_choose_1)
                        d_idx_1 = routes[r].index(d_choose_1)
                        # 交换
                        neighbor[r][p_idx_1] = p_choose_2
                        neighbor[r][d_idx_1] = d_choose_2
                    # 加入neighborhood
                    if neighbor not in neighborhood:
                        neighborhood.append(neighbor)
            cur_choose_num += 1
        
        if len(neighborhood) == 0:
            a = 1

        return neighborhood


class reverseInter():
    def __init__(self, instance=None, k=1): 
        self.instance = instance
        self.k = k # how many points reverse together

    def run(self, picking_solution):
        """ reverse route between two points randomly inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        # random get tow robot
        p1_list = []
        p2_list = []
        while len(p1_list) < 2 or len(p2_list) < 2:
            # random choose two robot
            """ TODO: relocate each robot rather than random two """
            r1, r2 = np.random.choice(range(len(routes)), 2, replace=False)
            # random choose one start point of this robot
            p1_list = [p for p in routes[r1] if self.instance.node2type[p] in ["P1", "P2"]]
            p2_list = [p for p in routes[r2] if self.instance.node2type[p] in ["P1", "P2"]]
        # 计算 p1_list 和 p2_list 的所有可能的片段长度
        p1_lengths = set(range(1, len(p1_list) + 1))
        p2_lengths = set(range(1, len(p2_list) + 1))
        common_lengths = p1_lengths.intersection(p2_lengths)
        # 遍历所有可能的相同长度的片段交换组合
        for length in common_lengths:
            for start_index_p1 in range(len(p1_list) - length + 1):
                for start_index_p2 in range(len(p2_list) - length + 1):
                    # 交换片段
                    new_p1_list = p1_list[:start_index_p1] + p2_list[start_index_p2:start_index_p2 + length] + p1_list[start_index_p1 + length:]
                    new_p2_list = p2_list[:start_index_p2] + p1_list[start_index_p1:start_index_p1 + length] + p2_list[start_index_p2 + length:]
                    neighbor = copy.deepcopy(routes)
                    for p1 in range(len(p1_list)):
                        # need exchange's p1 p2 d1 d2
                        p_choose_1 = p1_list[p1]
                        if p_choose_1 == new_p1_list[p1]:
                            continue
                        d_choose_1 = p_choose_1 + 2*self.instance.n
                        p_idx_1 = neighbor[r1].index(p_choose_1)
                        d_idx_1 = neighbor[r1].index(d_choose_1)
                        neighbor[r1][p_idx_1] = new_p1_list[p1]
                        neighbor[r1][d_idx_1] = new_p1_list[p1] + 2*self.instance.n
                    for p2 in range(len(p2_list)):
                        # need exchange's p1 p2 d1 d2
                        p_choose_2 = p2_list[p2]
                        if p_choose_2 == new_p2_list[p2]:
                            continue
                        d_choose_2 = p_choose_2 + 2*self.instance.n
                        p_idx_2 = neighbor[r2].index(p_choose_2)
                        d_idx_2 = neighbor[r2].index(d_choose_2)
                        neighbor[r2][p_idx_2] = new_p2_list[p2]
                        neighbor[r2][d_idx_2] = new_p2_list[p2] + 2*self.instance.n
                    # 加入neighborhood
                    if neighbor not in neighborhood:
                        neighborhood.append(neighbor)
        
        return neighborhood


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


class greedyRelocateInner1():
    def __init__(self, instance=None, k=1):
        """ greedy choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inner route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        p_list = []
        while len(p_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p_list = [p for p in routes[k] if self.instance.node2type[p] in ["P1", "P2"]]
        # 获取起终点
        p_choose = np.random.choice(p_list)
        d_choose = p_choose + 2*self.instance.n
        # 删除起终点
        routes[k].remove(p_choose)
        routes[k].remove(d_choose)
        # 根据chosen的pd来获取neighborhood
        route_point_num = len(routes[k])
        """ tips: pos_p < pos_d """
        for pos_p in range(1, route_point_num+1): # 不能插入第一个点
            for pos_d in range(pos_p, route_point_num+1): # d在p之后
                neighbor = copy.deepcopy(routes)
                # 插入p和d
                neighbor[k].insert(pos_p, p_choose)
                neighbor[k].insert(pos_d, d_choose)
                if neighbor not in neighborhood:
                    neighborhood.append(neighbor)

        return neighborhood


class greedyRelocateInter1():
    def __init__(self, instance=None, k=1):
        """ greedy choose some p and d pair and relocate their positions in different robots """
        self.instance = instance
        self.k = k # how many points relocate together, k=1:relocate, k>1:Or-Opt

    def run(self, picking_solution):
        """ relocate point and the point next to it randomly inter route (capacity not considered)

        Args:
            picking_solution (List[List[int]]): idxs of points of each route

        Returns:
            neighborhood (List[List[int]]): idxs of points of each route of each neighbor
        """
        routes = copy.deepcopy(picking_solution)
        neighborhood = [] # 邻域集合
        p_list = []
        while len(p_list) == 0:
            # random choose one robot
            """ TODO: relocate each robot rather than random one """
            k = np.random.randint(0, len(routes))
            # random choose one start point of this robot
            p_list = [p for p in routes[k] if self.instance.node2type[p] in ["P1", "P2"]]
        # 获取起终点
        p_choose = np.random.choice(p_list)
        d_choose = p_choose + 2*self.instance.n
        # 删除起终点
        routes[k].remove(p_choose)
        routes[k].remove(d_choose)
        # 根据chosen的pd来获取neighborhood
        route_num = len(routes)
        for r in range(route_num): # 选择第几个robot插入
            """ tips: pos_p < pos_d """
            route_point_num = len(routes[r])
            for pos_p in range(1, route_point_num): # 不能插入第一个点
                for pos_d in range(pos_p, route_point_num): # d在p之后
                    neighbor = copy.deepcopy(routes)
                    # 插入p和d
                    neighbor[r].insert(pos_p, p_choose)
                    neighbor[r].insert(pos_d, d_choose)
                    if neighbor not in neighborhood:
                        neighborhood.append(neighbor)

        return neighborhood