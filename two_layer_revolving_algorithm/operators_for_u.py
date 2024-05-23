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