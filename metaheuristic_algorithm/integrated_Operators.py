'''
File: integrated_Operators.py
Project: Integrated_Picking---Sorting_Model
File Created: Sunday, 7th May 2023 7:27:16 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''


import numpy as np
import utils


# base classes
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
    
    def cal_insert_cost(self, sequence, pos, item):
        """ 计算在序列 sequence 的 pos 位置插入 item 的成本 """
        if pos == len(sequence):
            return self.instance.disMatrix[sequence[-1], item]
        else:
            return self.instance.disMatrix[sequence[pos-1], item] + self.instance.disMatrix[item, sequence[pos]] - self.instance.disMatrix[sequence[pos-1], sequence[pos]]
    
    def cal_remove_cost(self, sequence, pos):
        """ 计算删除序列 sequence 的 pos 位置减少的成本 """
        if pos == len(sequence) - 1:
            return self.instance.disMatrix[sequence[pos-1], sequence[pos]]
        else:
            return self.instance.disMatrix[sequence[pos-1], sequence[pos]] + self.instance.disMatrix[sequence[pos], sequence[pos+1]] - self.instance.disMatrix[sequence[pos-1], sequence[pos+1]]

    def set(self, solution):
        # 获取邻域解
        raise NotImplementedError

class PickingRandomBreak(Operator):
    def __init__(self, instance, break_num=1):
        super().__init__(instance)
        self.break_num = break_num # 破坏任务数
        self.type = "picking"

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
        break_info = {
            "break_p_list": break_p_list,
            "break_d_list": break_d_list
        }
        return break_info

class PickingRandomRepair(Operator):
    def __init__(self, instance):
        super().__init__(instance)
        self.type = "picking"

    def set(self, solution, break_info):
        """ 随机修复 """
        break_p_list, break_d_list = break_info["break_p_list"], break_info["break_d_list"]
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
    def __init__(self, instance):
        super().__init__(instance)
        self.type = "picking"

    def set(self, solution, break_info):
        """ 贪婪修复 """
        break_p_list, break_d_list = break_info["break_p_list"], break_info["break_d_list"]
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
                    cost1 = self.cal_insert_cost(routes[k], p_pos, p_break)
                    self.safe_insert(routes[k], p_pos, p_break)
                    # 对于所有终点插入位置
                    for d_pos in range(p_pos+1, len(routes[k])+1):
                        cost2 = self.cal_insert_cost(routes[k], d_pos, d_break)
                        self.safe_insert(routes[k], d_pos, d_break)
                        # 计算插入该位置的成本
                        # cur_cost, _ = utils.efficient_integrated_evaluate(self.instance, routes, solution['sorting'])
                        # 简单计算成本
                        cur_cost = cost1 + cost2
                        if cur_cost < min_cost:
                            min_cost = cur_cost
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
                    

class SortingRandomBreak(Operator):
    def __init__(self, instance, break_num=1):
        super().__init__(instance)
        self.break_num = break_num
        self.type = "sorting"
    
    def set(self, solution):
        sorting = solution["sorting"] # order2picker
        break_o_list = np.random.choice(range(len(sorting)), self.break_num, replace=False)
        for o in break_o_list:
            sorting[o] = 0 #! default picker 0
        break_info = {
            "break_o_list": break_o_list.tolist()
        }
        return break_info

class SortingRandomRepair(Operator):
    def __init__(self, instance):
        super().__init__(instance)
        self.type = "sorting"

    def set(self, solution, break_info):
        break_o_list = break_info["break_o_list"]
        sorting = solution["sorting"] # order2picker        
        for o in break_o_list:
            p = np.random.randint(0, self.instance.P)
            sorting[o] = p
        
class SortingGreedyRepair(Operator):
    def __init__(self, instance):
        super().__init__(instance)
        self.type = "sorting"
    
    def backupon(self, best_solution, best_obj, cur_solution, break_o_list, cur_p):
        if cur_p >= len(break_o_list):
            cur_obj, _ = utils.efficient_integrated_evaluate(self.instance, cur_solution['picking'], cur_solution['sorting'])
            if cur_obj < best_obj:
                best_obj = cur_obj
                best_solution['sorting'] = cur_solution['sorting'].copy()
            return
        o = break_o_list[cur_p]
        for p in range(self.instance.P):
            cur_solution['sorting'][o] = p
            self.backupon(best_solution, best_obj, cur_solution, break_o_list, cur_p+1)

    def set(self, solution, break_info):
        best_solution = solution.copy()
        best_obj, _ = utils.efficient_integrated_evaluate(self.instance, best_solution['picking'], best_solution['sorting']) 
        cur_solution = solution.copy()
        break_o_list = break_info["break_o_list"]
        cur_p = 0
        self.backupon(best_solution, best_obj, cur_solution, break_o_list, cur_p)
        solution['sorting'] = best_solution['sorting'].copy()
        


