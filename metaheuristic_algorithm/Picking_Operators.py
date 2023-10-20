'''
File: Picking_Operators.py
Project: Integrated_Picking---Sorting_Model
File Created: Wednesday, 26th April 2023 8:06:44 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''


import numpy as np


class Operator:
    def __init__(self, picking_instance):
        self.node2type = picking_instance.node2type
        self.n = picking_instance.n

    def get(self, solution):
        # 获取邻域解
        raise NotImplementedError

    def run(self, solution):
        # 获取邻域
        raise NotImplementedError 


class RellocateD(Operator):
    """
    将终点插入同车的另一位置
    """
    def get(self, solution):
        neighbour = solution.copy()
        # 随机选择一辆车（非空）
        while True:
            k = np.random.randint(0, len(solution))
            route = solution[k]
            if len(route) > 1:
                break
        neighbour[k] = neighbour[k].copy()
        # 随机选择一个终点
        terminates = [i for i in range(1, len(route)) if self.node2type[route[i]] in ["D1", "D2"]] 
        ex_i = np.random.choice(terminates)
        # 提取
        tmp = neighbour[k].pop(ex_i)
        # 随机选择其起点后的一个点
        start_j = route.index(route[ex_i] - 2*self.n)
        ex_j = np.random.randint(start_j+1, len(route))
        # 插入
        neighbour[k].insert(ex_j, tmp)
        return neighbour
        
    def run(self, solution):
        neighbours = []
        for k in range(len(solution)):
            route = solution[k]
            for i in range(1, len(route)):
                if self.node2type[route[i]] == 'D1' or self.node2type[route[i]] == 'D2':
                    start_flag = False
                    for j in range(1, len(route)-1):
                        if start_flag: # if route_j after start of route_i
                            neighbour = solution.copy()
                            neighbour[k] = neighbour[k].copy()
                            tmp = neighbour[k].pop(i)
                            neighbour[k].insert(j, tmp)
                            neighbours.append(neighbour)
                        if route[j] == route[i] - 2*self.n: # route_j is start of route_i
                            start_flag = True 
        return neighbours


class RellocatePD(Operator):
    """
    将点插入其他位置，并调整终点
    """
    def get(self, solution):
        neighbour = solution.copy()
        # 随机选择一个起点（非空）
        while True:
            k1 = np.random.randint(0, len(solution)) 
            route1 = solution[k1]
            if len(route1) > 1:
                break
        neighbour[k1] = neighbour[k1].copy()
        starts = [i for i in range(1, len(route1)) if self.node2type[route1[i]] in ["P1", "P2"]]
        i1 = np.random.choice(starts)
        # 获取其终点
        j1 = neighbour[k1].index(route1[i1] + 2*self.n)
        # 提取
        node_i = neighbour[k1].pop(i1)
        node_j = neighbour[k1].pop(j1-1) # fix idx after insert i1
        # 随机选择一个起点插入位置
        k2 = np.random.randint(0, len(solution))
        route2 = solution[k2]
        neighbour[k2] = neighbour[k2].copy()
        i2 = np.random.randint(1, len(route2)+1)
        if i2 == len(route2):
            # 插入末尾
            neighbour[k2].append(node_i)
            neighbour[k2].append(node_j) 
        else:
            # 获取对应终点 
            j2 = -1
            for j2 in range(i2, len(route2)+1):
                if self.node2type[route2[j2]] == 'D1' or self.node2type[route2[j2]] == 'D2':
                    break
            # 插入
            neighbour[k2].insert(i2, node_i)
            if j2+1 == len(route2):
                neighbour[k2].append(node_j)
            else:
                neighbour[k2].insert(j2+1, node_j)
        return neighbour

    def run(self, solution):
        neighbours = []
        # 选择一个起点
        for k1 in range(len(solution)):
            route1 = solution[k1]
            # 跳过空路线
            if len(route1) <= 1:
                continue
            for i1 in range(1, len(route1)):
                if self.node2type[route1[i1]] == 'P1' or self.node2type[route1[i1]] == 'P2':
                    # 获取对应终点
                    j1 = route1.index(route1[i1] + 2*self.n)
                    # 将所选起终点提取出来，构成tmp_solution
                    tmp_solution = solution.copy()
                    tmp_solution[k1] = tmp_solution[k1].copy()
                    node_i1 = tmp_solution[k1].pop(i1)
                    node_j1 = tmp_solution[k1].pop(j1-1) # fix idx after insert i1
                    # 选择一个起点插入位置
                    for k2 in range(len(tmp_solution)):
                        route2 = tmp_solution[k2]
                        for i2 in range(1, len(route2)):
                            # 获取对应终点
                            j2 = -1
                            for j2 in range(i2, len(route2)):
                                if self.node2type[route2[j2]] == 'D1' or self.node2type[route2[j2]] == 'D2':
                                    break
                            # 插入
                            neighbour = tmp_solution.copy()
                            neighbour[k2] = neighbour[k2].copy()
                            neighbour[k2].insert(i2, node_i1)
                            neighbour[k2].insert(j2+1, node_j1) # fix idx after insert i1
                            neighbours.append(neighbour)
        return neighbours
                



