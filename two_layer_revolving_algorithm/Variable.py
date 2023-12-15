'''
File: Variable.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
Variable entity for the solution of each model or relaxed model.
Input the instance and create the Variable.
----------
Author: 626
Created Date: 2023.12.07
'''


import numpy as np



class Variable:
    def __init__(self, instance=None) -> None:
        # 主要决策变量
        self.y = np.zeros((instance.n, instance.P), dtype=int)
        self.z = np.zeros((instance.O, instance.P), dtype=int)
        # 辅助决策变量
        self.Q = np.zeros((instance.nodeNum))
        self.passX = np.zeros((instance.nodeNum, instance.robotNum), dtype=int)
        self.a1 = np.zeros((instance.O, instance.O), dtype=int)
        self.b1 = np.zeros((instance.O,instance.O), dtype=int)
        self.c1 = np.zeros((instance.O, instance.O), dtype=int)
        self.d1 = np.zeros((instance.O,instance.O), dtype=int)
        self.f = np.zeros((instance.n, instance.n, instance.P), dtype=int)
        # 时间决策变量
        self.tos = np.zeros((instance.O))
        self.toe = np.zeros((instance.O))
        self.T = np.zeros((instance.nodeNum))
        self.I = np.zeros(instance.n)
        self.Ta = np.zeros((instance.n, instance.P))
        self.Ts = np.zeros((instance.n, instance.P))
        self.Te = np.zeros((instance.n, instance.P))
        # 目标函数
        self.FT = 0

    def set_x_variable():
        pass

    def set_y_variable():
        pass

    def set_z_variable():
        pass

    def set_auxiliary_variable():
        pass

    def set_time_variable():
        pass




if __name__ == "__main__":
    pass
