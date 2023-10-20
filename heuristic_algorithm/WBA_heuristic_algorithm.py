'''
File: WBA_heuristic_algorithm.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a heuristic algorithm for the init solution of the sorting model.
----------
Author: 626
Created Date: 2023.10.19
'''


import Integrated_Instance
import numpy as np
import utils
from tqdm import tqdm
from Sorting_Instance import Instance


class WBA_heuristic_algorithm():

    def __init__(self, instance):
        self.N = instance.bins_num
        self.O = instance.O
        self.P = instance.P
        self.IO = instance.IO
    
    # input instances, output solution(chromosome)
    def WBA_main(self):
        car_chromosome = []
        totes_num = self.calculation_totes_num()
        # 任务分配
        stations_totes_num = [ 0 for i in range(self.P)]
        for i in range(self.O):
            insert_station = stations_totes_num.index(min(stations_totes_num))
            car_chromosome.append(insert_station)
            stations_totes_num[insert_station] += totes_num[i]
        
        z_val = self.tansfer_z(car_chromosome)

        return z_val
        
    def calculation_totes_num(self):
        totes_num = []
        for i in range(self.O):
            sum_totes = 0
            for j in range(self.N):
                if self.IO[j][i]:
                    sum_totes += 1
            totes_num.append(sum_totes)
        
        return totes_num
    
    def tansfer_z(self,car_chromosome):
        z_val = [[0] * self.P for i in range(self.O)]
        for i in range(len(car_chromosome)):
            z_val[i][car_chromosome[i]] = 1

        return z_val


# 建立sorting评估模型
def build_sorting_evaluate_model(instance, z_val):

    from Sorting_Gurobi_Model import Sorting_Gurobi
    import gurobipy as gp
    # 1. build model
    model_builder = Sorting_Gurobi(instance)
    model = gp.Model("Evaluate_Sorting_Model")
    # 2. set variables value
    info = model_builder.build_model(model)
    z = info["z"]
    for i in range(instance.O):
        for j in range(instance.P):
            if z_val[i][j] == 1:
                z[i,j].setAttr("LB", 1)
            elif z_val[i][j] == 0:
                z[i,j].setAttr("UB", 0)
    # 3. update model
    model.update()
    return model

# picking模型评估函数
def sorting_evaluate(instance, z_val):
    model = build_sorting_evaluate_model(instance, z_val) 
    model.setParam("OutputFlag", 0)
    model.optimize()
    if model.Status == 3:
        return 1e6
    else:
        return model.ObjVal


if __name__ == "__main__":
    # generate instance]
    w_num = 5
    l_num = 5
    bins_num = 20
    robot_num = 5
    picking_station_num = 5
    orders_num = 20
    instance = Integrated_Instance.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    WBA = WBA_heuristic_algorithm(instance)
    z_val = WBA.WBA_main()
    Obj = sorting_evaluate(Instance(0, bins_num, picking_station_num, orders_num), z_val)
    print(Obj)



    