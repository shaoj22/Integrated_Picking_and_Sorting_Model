'''
File: instance.py
Project: HaiSystem_picking_and_sorting_integrated
Description:
-----
Author: Stitch
Created Date: April 21th 2023
'''

import numpy as np
import matplotlib.pyplot as plt
from utils import DrawTools
import random
import Picking_Instance

class Instance(Picking_Instance.Instance):
    def __init__(self, w_num, l_num, bins_num, robot_num, picking_station_num, orders_num, seed=1):
        """__init__ generate instance

        Args:
            w_num (int): 宽度方向上块的个数 (y方向)
            l_num (int): 长度方向上块的个数 (x方向)
            task_num (int): 任务的个数
            seed (int, optional): 随机种子. Defaults to 1.
        """
        super().__init__(w_num, l_num, bins_num, robot_num)
        # add set conveyors params
        self.P = picking_station_num
        self.O = orders_num
        self.picking_time = 10
        self.v = 0.25
        self.bins_num = bins_num
        self.Dip, self.Dpi, self.IO, self.sumIO = self.generate_conveyors()

    def generate_conveyors(self):
        N = self.bins_num
        P = self.P
        O = self.O
        T1 = self.picking_time
        # 生成入口至拣选站的距离矩阵（固定长度为10）
        dip = []
        dpi = []
        x1 = 0
        for i in range(P):
            x1 += 10
            dip.append(x1)
            x2 = 110 - x1
            dpi.append(x2)
        Dip = [dip for _ in range(N)]
        # 生成出口至拣选站的距离矩阵
        Dpi = [dpi for _ in range(N)]
        # 初始化料箱i与订单o直接的关系（随机初始化）
        IO = [[0 for _ in range(O)] for _ in range(N)]
        for i in range(N):
            # 该料箱属于几个订单,n代表最多属于几个订单
            n = random.randint(1, 3)
            for j in range(n):
                m = random.randint(0, O - 1)
                IO[i][m] = 1
        # 初始化总的料箱任务数量
        sum1 = 0
        for i in range(len(IO)):
            sum1 += sum(IO[i])
        sumIO = sum1

        return Dip, Dpi, IO, sumIO




if __name__ == "__main__":
    # generate instance
    w_num = 2
    l_num = 2
    bins_num = 5
    robot_num = 3
    picking_station_num = 3
    orders_num = 5
    instance = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    print(instance.n)
    # print("generate {} tasks".format(5))
    # show map structure
    instance.map.render()