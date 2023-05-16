'''
File: integrated_Operators.py
Project: Integrated_Picking---Sorting_Model
File Created: Sunday, 7th May 2023 7:27:16 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''

import numpy as np
import utils

class Operator:
    def __init__(self, instance):
        # 获取算例信息
        self.instance = instance
        self.node2type = instance.node2type
        self.n = instance.n
    
    def safe_insert(self, sequence, pos, item):
        """ 在序列 sequence 的 pos 位置插入 item """
        if pos == len(sequence):
            sequence.append(item)
        else:
            sequence.insert(pos, item)

    def set(self, solution):
        # 获取邻域解
        raise NotImplementedError

class PickingRandomBreak(Operator):
    def __init__(self, instance, break_num=1):
        super().__init__(instance)
        self.break_num = break_num # 破坏任务数

    def set(self, solution):
        """ 随机破坏 break_num 个任务（选择起点，取出其起点和终点） """
        routes = solution["picking"]
        break_p_list = []
        break_d_list = []
        cur_break_num = 0
        while cur_break_num < self.break_num:
            # 随机选择一辆车
            k = np.random.randint(0, self.instance.robotNum)
            # 随机选择一个起点
            p_list = [p for p in routes[k] if self.node2type[p] in ["P1", "P2"]]
            if len(p_list) == 0:
                continue
            p_break = np.random.choice(p_list)
            # 获取其对应终点
            d_break = p_break + 2*self.n
            # 删除起终点
            routes[k].remove(p_break)
            routes[k].remove(d_break)
            # 保存删除的起终点
            break_p_list.append(p_break)
            break_d_list.append(d_break)
            cur_break_num += 1
        # 返回删除的起终点
        return break_p_list, break_d_list

class PickingRandomRepair(Operator):
    def set(self, solution, break_p_list, break_d_list):
        """ 随机修复 """
        routes = solution["picking"]
        break_num = len(break_p_list)
        for i in range(break_num):
            p_break = break_p_list[i]
            d_break = break_d_list[i]
            # 选择一辆车
            k = np.random.randint(0, self.instance.robotNum)
            # 选择一个位置
            pos = np.random.randint(1, len(routes[k])+1)
            # 插入起终点
            self.safe_insert(routes[k], pos, d_break)
            self.safe_insert(routes[k], pos, p_break)

class PickingGreedyRepair(Operator):
    def set(self, solution, break_p_list, break_d_list):
        """ 贪婪修复 """
        routes = solution["picking"]
        break_num = len(break_p_list)
        for i in range(break_num):
            p_break = break_p_list[i]
            d_break = break_d_list[i]
            min_cost = np.inf
            # 对于所有车
            for k in range(self.instance.robotNum):
                # 对于所有起点插入位置
                for p_pos in range(1, len(routes[k])+1):
                    self.safe_insert(routes[k], p_pos, p_break)
                    # 对于所有终点插入位置
                    for d_pos in range(p_pos+1, len(routes[k])+1):
                        self.safe_insert(routes[k], d_pos, d_break)
                        # 计算插入该位置的成本
                        cur_cost = utils.efficient_integrated_insert_cost(self.instance, routes, solution['sorting'])
                        if cost < min_cost:
                            min_cost = cost
                            min_k = k
                            min_p_pos = p_pos
                            min_d_pos = d_pos
                        # 恢复
                        routes[k].pop(d_pos)
                    # 恢复
                    routes[k].pop(p_pos)
            # 插入最优位置
            self.safe_insert(routes[min_k], min_p_pos, p_break)
            self.safe_insert(routes[min_k], min_d_pos, d_break)
            assert utils.efficient_integrated_insert_cost(self.instance, routes, solution['sorting']) == min_cost
                    
