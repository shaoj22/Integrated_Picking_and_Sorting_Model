'''
File: tests.py
Project: Integrated_Picking---Sorting_Model
File Created: Monday, 24th April 2023 10:42:23 am
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''
from Integrated_Gurobi_Model import *
import numpy as np
import utils

def test_evaluate():
    w_num = 3
    l_num = 3
    bins_num = 5
    robot_num = 3
    picking_station_num = 2
    orders_num = 2
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    alg = Integrated_Gurobi_Model(instance = problem, time_limit = 3600)
    model, Obj, Time, objBound, SolutionT, SolutionI, SolutionTo = alg.run_gurobi()
    print("\nmodel obj = {}, time_cost = {}".format(Obj, Time))
    # get solution
    x_val = np.zeros((problem.nodeNum, problem.nodeNum, problem.robotNum))
    for i in range(problem.nodeNum):
        for j in range(problem.nodeNum):
            for k in range(problem.robotNum):
                x_val[i][j][k] = model.getVarByName(f'x[{i},{j},{k}]').x
    y_val = np.zeros((problem.n, picking_station_num))
    for i in range(problem.n):
        for j in range(picking_station_num):
            y_val[i][j] = model.getVarByName(f'y[{i},{j}]').x
    z_val = np.zeros((orders_num, picking_station_num))
    for i in range(orders_num):
        for j in range(picking_station_num):
            z_val[i][j] = model.getVarByName(f'z[{i},{j}]').x
    # evaluate solution
    start = time.time()
    obj = utils.evaluate(problem, x_val, y_val, z_val)
    end = time.time()
    print("evaluate obj = {}, time_cost = {}".format(obj, end - start))

if __name__ == "__main__":
    test_evaluate()


