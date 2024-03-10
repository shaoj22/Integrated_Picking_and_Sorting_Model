'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-10 10:58:36
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-03-10 21:54:00
FilePath: \Integrated_Picking_and_Sorting_Model\test_fitness.py
Description: test the fitness function.
'''


import sys
sys.path.append('..')
import numpy as np
from gurobi_model.Integrated_Gurobi_Model import Integrated_Gurobi_Model
from generate_instances.Integrated_Instance import Instance
from two_layer_revolving_algorithm.Variable import Variable
import utils
import utils_new
from metaheuristic_algorithm.Integrated_ALNS import ALNS
from two_layer_revolving_algorithm.common_algorithm_by_strengthened_gurobi import commonAlgorithmByStrengthenedGurobi




if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 45
    robot_num = 10
    picking_station_num = 10
    orders_num = 35
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    # run ALNS
    alg = ALNS(problem, iter_num=2000)
    solution, obj, obj_of_500 = alg.run()
    picking_solution = solution['picking']
    sorting_solution = solution['sorting']
    print("test the final solution:\n\n\n\n\n")
    obj, info = utils.efficient_integrated_evaluate(integrated_instance=problem, picking_solution=picking_solution, sorting_solution=sorting_solution)
    # 把有效评估改成使用common algorithm进行评估
    variable = Variable(problem)
    # 把xyz赋值给variable
    x_val, y_val, z_val = utils.solution_transfer(problem, picking_solution, sorting_solution)
    variable_dict = {
        'x' : x_val,
        'y' : y_val,
        'z' : z_val
    }
    variable.set_x_variable(variable_dict)
    variable.set_y_variable(variable_dict)
    variable.set_z_variable(variable_dict)
    # 模型评估
    solver = commonAlgorithmByStrengthenedGurobi(problem, variable)
    model = solver.run_gurobi_model()
    if model is not None and model.status == 2:
        model_obj = model.objVal
    else:
        model_obj = float('inf')
    
    # get model's solution
    Ta = np.zeros((problem.n, problem.P))
    Te = np.zeros((problem.n, problem.P))
    for i in range(problem.n):
        for j in range(problem.P):
            Ta[i][j] = model.getVarByName(f'Ta[{i},{j}]').x
            Te[i][j] = model.getVarByName(f'Te[{i},{j}]').x
    # check the solution
    # for tote in range(len(info['Tip_arrive'])):
    #     print("1.1 ALNS第{}个tote到达各拣选站的时间:{}".format(tote, info['Tip_arrive'][tote]))
    #     print("1.2 model第{}个tote到达各拣选站的时间:{}".format(tote, Ta[tote]))
    #     print("2.1 ALNS第{}个tote离开各拣选站的时间:{}".format(tote, info['Tip_leave'][tote]))
    #     print("2.2 model第{}个tote离开各拣选站的时间:{}".format(tote, Te[tote]))


    # gurobi
    gurobi_obj, gurobi_info = utils.integrated_evaluate(problem, x_val, z_val)

    # 输出点的T
    # for i in range(len(gurobi_info['T'])):
    #     print("ALNS第{}个点的T:{}".format(i, gurobi_info['T'][i]))


    new_obj, new_info = utils_new.efficient_integrated_evaluate(integrated_instance=problem, picking_solution=picking_solution, sorting_solution=sorting_solution)
    # 输出车的路径和到达该点的时间
    for i in range(len(picking_solution)):
        # 统计该车到达各点的时间
        gurobi_car_t = []
        ALNS_car_t = []
        new_utils_car_t = []
        for j in range(len(picking_solution[i])):
            gurobi_car_t.append(gurobi_info['T'][picking_solution[i][j]])
            ALNS_car_t.append(info['passTime'][picking_solution[i][j]])
            new_utils_car_t.append(new_info['T'][picking_solution[i][j]])
        print("ALNS第{}个车的路径:{}".format(i, picking_solution[i]))
        print("gurobi第{}个车的时间{}".format(i, gurobi_car_t))
        print("ALNS第{}个车的时间{}".format(i, ALNS_car_t))
        print("new_utils第{}个车的时间{}".format(i, new_utils_car_t))
    
    print("ALNS ev solution: ", obj)
    print("new_utils ev solution: ", new_obj)
    print("model ev solution: ", model_obj)
    




