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
        self.x = np.zeros((instance.nodeNum, instance.nodeNum), dtype=int)
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

    def set_x_variable(self, variable_update_dict):
        """ input the variable dict from the solved gurobi model and get x variable
        
        Args:
            variable_update_dict (dict): from TRA_utils get variable from solved model
        """
        self.x = variable_update_dict['x']

    def set_y_variable(self, variable_update_dict):
        self.y = variable_update_dict['y']

    def set_z_variable(self, variable_update_dict):
        self.z = variable_update_dict['z']

    def set_auxiliary_variable(self, variable_update_dict):
        self.a1 = variable_update_dict['a1']
        self.b1 = variable_update_dict['b1']
        self.c1 = variable_update_dict['c1']
        self.d1 = variable_update_dict['d1']
        self.Q = variable_update_dict['Q']
        self.passX = variable_update_dict['passX']
        self.f = variable_update_dict['f']

    def set_time_variable(self, variable_update_dict):
        self.tos = variable_update_dict['tos']
        self.toe = variable_update_dict['toe']
        self.I = variable_update_dict['I']
        self.Ta = variable_update_dict['Ta']
        self.I = variable_update_dict['Ts']
        self.Ta = variable_update_dict['Te']
        self.Ta = variable_update_dict['T']
        self.Ta = variable_update_dict['FT']

if __name__ == "__main__":
    pass
