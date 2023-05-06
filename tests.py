'''
File: tests.py
Project: Integrated_Picking---Sorting_Model
File Created: Monday, 24th April 2023 10:42:23 am
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''
import Integrated_Gurobi_Model
import Picking_Gurobi_Model
import numpy as np
import time
import utils

def test_integrated_evaluate():
    w_num = 3
    l_num = 3
    bins_num = 5
    robot_num = 3
    picking_station_num = 2
    orders_num = 2
    problem = Integrated_Gurobi_Model.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    alg = Integrated_Gurobi_Model.Integrated_Gurobi_Model(instance = problem, time_limit = 3600)
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
    obj = utils.integrated_evaluate(problem, x_val, y_val, z_val)
    end = time.time()
    print("evaluate obj = {}, time_cost = {}".format(obj, end - start))

def test_picking_evaluate():
    w_num = 3
    l_num = 3
    task_num = 5
    robot_num = 2
    instance = Picking_Gurobi_Model.Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_Gurobi_Model.Picking_Gurobi_Model(instance = instance, time_limit = 3600)
    result_info = alg.run_gurobi()
    model = result_info["model"]
    # get solution
    x_val = np.zeros((instance.nodeNum, instance.nodeNum, instance.robotNum))
    for i in instance.N:
        for j in instance.N:
            for k in instance.K:
                x_val[i][j][k] = model.getVarByName(f'x[{i},{j},{k}]').x
    # evaluate solution
    start = time.time()
    obj = utils.picking_evaluate(instance, x_val)
    end = time.time()
    print("evaluate obj = {}, time_cost = {}".format(obj, end - start))

def test_picking_integrated_evluate():
    w_num = 3
    l_num = 3
    task_num = 5
    robot_num = 2
    picking_instance = Picking_Gurobi_Model.Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_Gurobi_Model.Picking_Gurobi_Model(instance = picking_instance, time_limit = 3600)
    result_info = alg.run_gurobi()
    model = result_info["model"]
    # get solution
    x_val = np.zeros((picking_instance.nodeNum, picking_instance.nodeNum))
    for i in picking_instance.N:
        for j in picking_instance.N:
            x_val[i][j] = model.getVarByName(f'x[{i},{j}]').x
    # evaluate solution
    pickers_num = 10
    orders_num = 5
    integrated_instance = Integrated_Gurobi_Model.Instance(w_num, l_num, task_num, robot_num, pickers_num, orders_num)
    start = time.time()
    obj = utils.picking_integrated_evaluate(integrated_instance, x_val)
    end = time.time()
    print("evaluate obj = {}, time_cost = {}".format(obj, end - start))

def test_picking_integrated_gap():
    w_num = 3
    l_num = 3
    task_num = 20
    robot_num = 10
    picking_instance = Picking_Gurobi_Model.Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_Gurobi_Model.Picking_Gurobi_Model(instance = picking_instance, time_limit = 3600)
    result_info = alg.run_gurobi()
    model = result_info["model"]
    # get solution
    x_val = np.zeros((picking_instance.nodeNum, picking_instance.nodeNum))
    for i in picking_instance.N:
        for j in picking_instance.N:
            x_val[i][j] = model.getVarByName(f'x[{i},{j}]').x
    # evaluate picking solution
    pickers_num = 10
    orders_num = 5
    integrated_instance = Integrated_Gurobi_Model.Instance(w_num, l_num, task_num, robot_num, pickers_num, orders_num)
    obj1 = utils.picking_integrated_evaluate(integrated_instance, x_val)
    # get integrated solution
    alg = Integrated_Gurobi_Model.Integrated_Gurobi_Model(instance = integrated_instance, time_limit = 3600)
    result_list = alg.run_gurobi()
    obj2 = result_list[1]
    print("picking result = {}, integrated result = {}, picking integrated gap = {}".format(obj1, obj2, (obj1 - obj2) / obj1))


if __name__ == "__main__":
    # test_integrated_evaluate()
    # test_picking_evaluate()
    # test_picking_integrated_evluate()
    test_picking_integrated_gap()


