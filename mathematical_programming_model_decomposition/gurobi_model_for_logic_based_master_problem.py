'''
File: master_problem.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
relaxed all variable except x.
----------
Author: 626
Created Date: 2023.12.19
'''


import sys
sys.path.append('..')
import numpy as np
import gurobipy as gp
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from gurobipy import GRB
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.variable import Variable
import Integrated_Picking_and_Sorting_Model.utils as utils


class MasterProblem:
    def __init__(self, variable, time_limit=None, init_flag=False):
        """
        init the xRelaxedGurobiModel with inputting instance and variable.

        Args:
        variable (class): all variable num of the problem.
        """
        self.spn = [i for i in range(variable.spn)] # 子问题数量
        self.input_st = variable.input_st
        self.T_list = variable.T_list
        self.time_limit = time_limit
        self.init_flag = init_flag

    
    def build_gurobi_model(self, model):
        """ build gurobi model with obj and cons """
        # 添加决策变量
        st_list = [i for i in self.spn]
        st = model.addVars(st_list, vtype=GRB.BINARY, name="st")
        # 添加目标函数
        model.modelSense = GRB.MINIMIZE
        model.setObjective(gp.quicksum(st[i]*self.T_list[i] for i in st_list))
        # 添加约束条件
        model.addConstr( gp.quicksum(st[i] for i in self.spn) == 1)
        model.addConstrs( st[i] <= 1 - self.input_st[i] for i in self.spn)
        model.update()

    def run_gurobi_model(self):
        model = gp.Model("master_problem") # 创建gurobi模型
        self.build_gurobi_model(model) # 构建gurobi模型
        if self.time_limit is not None: # 求解时间限制
            model.setParam("TimeLimit", self.time_limit)
        model.setParam("OutputFlag", 0)  # 求解过程展示
        if self.init_flag: # 设置gurobi模型初始解
            self.set_init_solution(model)
        model.optimize() # 求解模型
        # 获得最优解
        if model.status == 2:
            solution_st = []
            for i in self.spn:
                var_name = f"st[{i}]"
                st_i = model.getVarByName(var_name).x
                solution_st.append(st_i)

        return solution_st


if __name__ == "__main__":
    w_num = 6
    l_num = 6
    bins_num = 5
    robot_num = 5
    picking_station_num = 5
    orders_num = 5
    problem = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    # 获得初始解
    picking_alg = NNH_heuristic_algorithm(problem)
    picking_solution = picking_alg.NNH_main()
    sorting_solution = [np.random.randint(problem.P) for _ in range(problem.O)]
    # 构造初始variable
    variable = Variable(problem)
    x_val, y_val, z_val = utils.solution_transfer(problem, picking_solution, sorting_solution)
    variable_dict = {
            'x' : x_val,
            'y' : y_val,
            'z' : z_val
        }
    variable.set_x_variable(variable_dict)
    variable.set_y_variable(variable_dict)
    variable.set_z_variable(variable_dict)
    variable.T_list = [100, 140, 130]
    variable.input_st = [1, 0, 0]
    # 求解
    solver = MasterProblem(variable)
    st = solver.run_gurobi_model()
    print(st)
