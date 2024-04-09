'''
File: common_algorithm_by_strengthened_gurobi.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a algorithm to get the objective of the model, after fixed x\y\z variable, and some variable that can be cal by x\y\z.
----------
Author: 626
Created Date: 2024.01.04
'''


import sys
sys.path.append('..')
import numpy as np
import gurobipy as gp
import time
import Integrated_Picking_and_Sorting_Model.utils
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.integrated_gurobi_model_update import IntegratedGurobiModel
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.TRA_utils import *
from gurobipy import GRB
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.Variable import Variable
# from metaheuristic_algorithm.Integrated_ALNS import ALNS
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm.TRA_utils import *

class commonAlgorithmByStrengthenedGurobi:
    def __init__(self, integrated_instance, Variable, time_limit=None, init_flag=False):
        """
        init the xyzRelaxedGurobiModel with inputting instance and Variable.

        Args:
        integrated_instance (class): instance class of the problem. 
        Variable (class): all Variable num of the problem.
        """
        # variable
        self.Variable = Variable
        # common param
        self.integrated_instance = integrated_instance
        self.time_limit = time_limit
        self.bigM = 10000
        # picking param
        self.init_flag = init_flag
        self.Q = integrated_instance.capacity
        self.delta_T = integrated_instance.pick_time
        self.N = list(range(integrated_instance.nodeNum))
        self.nodes = integrated_instance.nodes
        self.n = integrated_instance.n
        self.P1 = integrated_instance.P1
        self.P2 = integrated_instance.P2
        self.D1 = integrated_instance.D1
        self.D2 = integrated_instance.D2
        self.W = integrated_instance.W
        self.K = list(range(integrated_instance.robotNum))
        self.disMatrix = integrated_instance.disMatrix
        self.timeMatrix = integrated_instance.timeMatrix
        # sorting param
        self.P = integrated_instance.P
        self.Dip = integrated_instance.Dip
        self.Dpi = integrated_instance.Dpi
        self.v = integrated_instance.v
        self.O = integrated_instance.O
        self.IO = integrated_instance.IO
        self.picking_time = integrated_instance.picking_time
        self.queue_length = integrated_instance.queue_length
        self.sumIO = integrated_instance.sumIO


    def cal_passX_variable():
        pass


    def build_gurobi_model(self, model):
        """ build gurobi model with obj and cons """
        M = self.bigM


        # 添加决策变量
        
        """
        x_list = [(i,j) for i in self.N for j in self.N]
        x = model.addVars(x_list, vtype=GRB.BINARY, name="x") # 车的路径
        y_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        y = model.addVars(y_list, vtype=GRB.BINARY, name="y") # 料箱i是否被分配给了拣选站p
        z_list = [(o,p) for o in range(self.O) for p in range(self.P)]
        z = model.addVars(z_list, vtype=GRB.BINARY, name="z") # 订单o是否被分配给了拣选站p
        """

        # 订单任务分拨上墙的决策变量

        tos_list = [ o for o in range(self.O)]
        tos = model.addVars( tos_list, vtype=GRB.CONTINUOUS, name="tos")
        toe_list = [ o for o in range(self.O)]
        toe = model.addVars( toe_list, vtype=GRB.CONTINUOUS, name="toe")
        a1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        a1 = model.addVars(a1_list, vtype=GRB.BINARY, name="a1")
        b1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        b1 = model.addVars(b1_list, vtype=GRB.BINARY, name="b1")
        c1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        c1 = model.addVars(c1_list, vtype=GRB.BINARY, name="c1")
        d1_list = [(o1,o2) for o1 in range(self.O) for o2 in range(self.O)]
        d1 = model.addVars(d1_list, vtype=GRB.BINARY, name="d1")

        # picking 决策变量
        
        # Q_list = [i for i in self.N]
        # Q = model.addVars( Q_list, vtype=GRB.CONTINUOUS, name="Q")  # 车的载重
        T_list = [i for i in self.N]
        T = model.addVars( T_list, vtype=GRB.CONTINUOUS, name="T")  # 车的时间
        # pass_list = [(i,k) for i in self.N for k in self.K]
        # passX = model.addVars( pass_list, vtype=GRB.BINARY, name="passX")  # 车k是否经过点i

        # sorting 决策变量

        I_list = [ i for i in range(self.n)]
        I = model.addVars( I_list, vtype=GRB.INTEGER, name="I")  # 料箱i的初始到达输送机的时间
        Ta_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Ta = model.addVars(Ta_list, vtype=GRB.INTEGER, name="Ta") # 料箱i到达拣选站p的时间
        Ts_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        Ts = model.addVars(Ts_list, vtype=GRB.INTEGER, name="Ts")  # 料箱i在拣选站p的开始拣选时间
        Te_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Te = model.addVars(Te_list, vtype=GRB.INTEGER, name="Te" ) # 料箱i在拣选站p的结束拣选时间
        f_list = [(i,j,p) for i in range(self.n) for j in range(self.n) for p in range(self.P)]
        f = model.addVars( f_list, vtype=GRB.BINARY, name="f") # 料箱i是否先于料箱j到达拣选站p


        FT = model.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间

        # 添加目标函数
        model.modelSense = GRB.MINIMIZE
        model.setObjective( FT ) # 最小化最大完成时间目标
        # model.setObjective( gp.quicksum(T[i] for i in self.N) ) # 最小化最大完成时间目标


        # 添加约束条件
        # 0. 最大完成时间约束
        model.addConstrs( FT >= T[i] for i in self.N)
       
        # 6. 时间约束
        model.addConstrs( T[j] >= T[i] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"] - M * (1 - self.Variable.x[i, j]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        model.addConstrs( T[i] >= self.nodes[i]["readyTime"] for i in self.N )
        model.addConstrs( T[i] <= self.nodes[i]["dueTime"] for i in self.N )
        # 8. 到达P2的时间>=到达环形输送机出口的时间
        model.addConstrs( T[i - self.n] >= Te[i - 2 * self.n, self.P-1] + (self.Dpi[i - 2 * self.n][self.P-1]/self.v) for i in self.D1 )
        # 9. 到达输送机的时间要>=到达D1的时间
        model.addConstrs( I[i] == T[i + 2 * self.n] for i in range(self.n) )
        # 10. 控制决策变量f的两条约束
        model.addConstrs( Ta[i,p] - Ta[j,p] <= (1 - f[i,j,p]) * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        model.addConstrs( Ta[i,p] - Ta[j,p] >= -f[i,j,p] * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 11. 先到达拣选站p的料箱i的结束拣选时间要小于等于后到达拣选站p的料箱j的开始拣选时间
        model.addConstrs( Ts[j,p] - Te[i,p] >= M * (f[i,j,p] + self.Variable.y[i,p] + self.Variable.y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        model.addConstrs( Ts[i,p] - Te[j,p] >= M * ((1- f[i,j,p]) + self.Variable.y[i,p] + self.Variable.y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 12. 只有订单o被分配给了拣选站p & 料箱i属于订单o，那么料箱i一定能被分配给拣选站p
        # model.addConstrs( self.IO[i][o] * self.Variable.z[o,p] <= self.Variable.y[i,p] for i in range(self.n) for p in range(self.P) for o in range(self.O))
        # 13. 所有任务都要被完成
        # model.addConstrs( gp.quicksum( self.Variable.z[o,p] for p in range(self.P)) == 1 for o in range(self.O))
        # model.addConstr( gp.quicksum( self.Variable.y[i,p] for i in range(self.n) for p in range(self.P)) == self.sumIO)
        # 14. 料箱i的结束拣选时间一定大于等于它的开始拣选时间（当yip=0时）
        model.addConstrs( Te[i,p] == Ts[i,p] + self.picking_time * self.Variable.y[i,p] for i in range(self.n) for p in range(self.P))
        # 15. 当料箱i不去拣选站p时，Ts=Ta
        model.addConstrs( Ts[i,p] <= Ta[i,p] + M * self.Variable.y[i,p] for i in range(self.n) for p in range(self.P))
        # 16. 料箱i到达第一个拣选站p=0时的时间（初始化）：
        model.addConstrs( Ta[i,0] == I[i] + self.Dip[i][0]/self.v for i in range(self.n))
        # 17. 料箱到达下一个拣选站的时间为：在上一个拣选站结束拣选的时间+路程时间（----------标记）约束条件3和约束条件2二选一就可以，到底是大于等于还是等于？
        model.addConstrs( Ta[i,p] == Te[i,p-1] + (self.Dip[i][p] - self.Dip[i][p-1])/self.v for i in range(self.n) for p in range(1,self.P))
        # 18. 开始拣选时间的约束——料箱i在p开始拣选的时间一定大于or等于在上一个拣选站结束拣选的时间+路程时间
        model.addConstrs( Ts[i, p] >= Ta[i, p] for i in range(self.n) for p in range(self.P))
        # 19. 拣选站处的缓存区大小约束
        model.addConstrs( Ts[i,p] - Ta[i,p] <= (self.queue_length-1) * self.picking_time for i in range(self.n) for p in range(self.P))
        # 20. 添加订单任务分拨上墙的约束条件
        model.addConstrs( tos[o1] - tos[o2] <= a1[o1,o2] * self.bigM for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( toe[o2] - tos[o1] <= b1[o1,o2] * self.bigM for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( tos[o] <= toe[o] for o in range(self.O))
        model.addConstrs( c1[o1,o2] >= a1[o1,o2] + b1[o1,o2] - 1 for o1 in range(self.O) for o2 in range(self.O))
        model.addConstrs( d1[o1,o2] >= c1[o1,o2] + self.Variable.z[o1,p] +self.Variable.z[o2,p] - 2 for o1 in range(self.O) for o2 in range(self.O) for p in range(self.P))
        model.addConstrs( gp.quicksum( d1[o1,o2] for o2 in range(self.O)) <= 8 for o1 in range(self.O))
        # 添加订单任务分拨上墙与任务分配和机器人调度的衔接约束条件
        model.addConstrs( toe[o] - Te[i,p] >= (self.IO[i][o] + self.Variable.y[i,p] + self.Variable.z[o,p] - 3) * self.bigM for o in range(self.O) for i in range(self.n) for p in range(self.P))
        model.addConstrs( tos[o] - Ts[i,p] <= (3 - self.IO[i][o] - self.Variable.y[i,p] - self.Variable.z[o,p]) * self.bigM for o in range(self.O) for i in range(self.n) for p in range(self.P))
        model.update()


    def run_gurobi_model(self):
        model = gp.Model("IntegratedGurobiModel") # 创建gurobi模型
        self.build_gurobi_model(model) # 构建gurobi模型
        if self.time_limit is not None: # 求解时间限制
            model.setParam("TimeLimit", self.time_limit)
        model.setParam("OutputFlag", 0)  # 求解过程展示
        if self.init_flag: # 设置gurobi模型初始解
            self.set_init_solution(model)
        model.optimize() # 求解模型

        return model
    
    def efficient_integrated_evaluate(self, integrated_instance, picking_solution, sorting_solution, variable_dict):
        obj = 0
        instance = integrated_instance
        # calculate pass time of each node and check time_windows
        timeMatrix = instance.timeMatrix
        passTime = np.zeros(instance.nodeNum)
        node2ki = {}
        P2_list = []
        for ri, route in enumerate(picking_solution):
            cur_time = 0
            # print("当前的路径为：", route)
            for i in range(1, len(route)):
                cur_time += instance.nodes[route[i-1]]["serviceTime"] + timeMatrix[route[i-1], route[i]]
                wait_time = max(cur_time - instance.nodes[route[i]]["dueTime"], 0)
                cur_time = max(cur_time, instance.nodes[route[i]]["readyTime"])
                if cur_time > instance.nodes[route[i]]["dueTime"]:
                    obj += 10000
                passTime[route[i]] = cur_time
                # print("到达{}点的时间为{},等待时间为{}, 模型评估出来的时间为{}, 行驶时间为{}, 服务时间为{}".format(route[i], cur_time, wait_time, variable_dict['T'][route[i]], timeMatrix[route[i-1], route[i]], instance.nodes[route[i-1]]["serviceTime"]))
                node2ki[route[i]] = (ri, i)
                if instance.node2type[route[i]] == "P2":
                    P2_list.append(route[i])
        # calculate pick_time of each task 
        Tip_arrive = np.zeros((instance.n, instance.P))
        Tip_leave = np.zeros((instance.n, instance.P))
        ## 计算进入环形输送机的时间
        enter_time = np.zeros(instance.n)
        for ni in range(instance.n):
            d1 = ni + 2 * instance.n
            enter_time[ni] = passTime[d1]
        ## 计算到达各个拣选站的时间
        for p in range(instance.P):
            if p == 0: # 拣选站1的到达时间
                Tip_arrive[:, 0] = enter_time + np.array(instance.Dip)[:, 0] / instance.v 
            else: # 根据拣选站p-1的离开时间更新拣选站p的到达时间
                Tip_arrive[:, p] = Tip_leave[:, p-1] + instance.distance_between_pickers / instance.v
            # 根据拣选站p的到达时间更新料箱ni的离开时间
            ni_list = [ni for ni in range(instance.n)]
            sorted_ni_list = sorted(ni_list, key=lambda x: Tip_arrive[x, p])
            cur_time = 0
            for ni in sorted_ni_list:
                # 检查料箱ni是否属于拣选站p
                belong_flag = False
                for o in range(instance.O):
                    if instance.IO[ni][o] and sorting_solution[o] == p:
                        # 如果料箱ni属于订单o且订单o被分配给第一个拣选站
                        belong_flag = True
                        break
                if belong_flag:
                    Tip_leave[ni, p] = max(cur_time, Tip_arrive[ni, p]) + instance.picking_time
                    cur_time = Tip_leave[ni, p]
                    # 检查队列长度
                    if Tip_leave[ni, p] - Tip_arrive[ni, p] > (instance.queue_length-1) * instance.picking_time:
                        obj += 10000
                else:
                    Tip_leave[ni, p] = Tip_arrive[ni, p]
        ## 计算出唤醒输送机的时间
        out_time = Tip_leave[:, instance.P-1] + np.array(instance.Dpi)[:, instance.P-1] / instance.v
        delta_T_list = out_time - enter_time
                # check feasibility and fix passTime of P2, D2
        # 1. D12 later than P12
        for ni in range(2*instance.n): # for P12 
            if passTime[ni] > passTime[ni+2*instance.n]:
                obj += 10000
        # 2. P2 later than D1, and fix pick_time
        for ni in P2_list:
            if passTime[ni] < passTime[ni+instance.n] + delta_T_list[ni-instance.n]:
                k, start_i = node2ki[ni]
                route = picking_solution[k]
                extra_time = passTime[ni+instance.n] + delta_T_list[ni-instance.n] - passTime[ni]
                for i in range(start_i, len(route)):
                    passTime[route[i]] += extra_time
        # 3. check capacity
        for k in range(instance.robotNum):
            load = 0
            for i in range(1, len(picking_solution[k])):
                load += instance.nodes[picking_solution[k][i]]["demand"]
                if load > instance.capacity:
                    obj += 10000
        obj += np.max(passTime)
        # print("passTime :\n", passTime)
        # print("Tip_arrive (Ta) :\n", Tip_arrive)
        # print("Tip_leave (Te) :\n", Tip_leave)

        info = {"passTime" : passTime, "Tip_arrive" : Tip_arrive, "Tip_leave" : Tip_leave}

        
        # check the time 
        for ri, route in enumerate(picking_solution):
            print("当前的路径为：", route)
            for i in range(1, len(route)):
                # print("到达{}点的时间为{},等待时间为{}, 模型评估出来的时间为{}, 行驶时间为{}, 服务时间为{}".format(route[i], passTime[route[i]], max(passTime[route[i]] - instance.nodes[route[i]]["dueTime"], 0), variable_dict['T'][route[i]], timeMatrix[route[i-1], route[i]], instance.nodes[route[i-1]]["serviceTime"]))
                pass

        return obj, info

    


if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 20
    robot_num = 5
    picking_station_num = 5
    orders_num = 10
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    solver = IntegratedGurobiModel(problem, time_limit=20)
    model = solver.run_gurobi_model()
    # print(model.objVal)
    # 获取初始解
    input_variable = Variable(problem)
    # variable_dict0, is_solved0 = get_variable_from_solved_model(input_variable, ['x', 'y', 'z'], model)
    # input_variable.set_x_variable(variable_dict0)
    # input_variable.set_y_variable(variable_dict0)
    # input_variable.set_z_variable(variable_dict0)
    alg = ALNS(problem, iter_num=1000)
    solution, obj = alg.run()
    x_val, y_val, z_val = utils.solution_transfer(problem, solution['picking'], solution['sorting'])
    variable_dict = {
            'x' : x_val,
            'y' : y_val,
            'z' : z_val
        }
    input_variable.set_x_variable(variable_dict)
    input_variable.set_y_variable(variable_dict)
    input_variable.set_z_variable(variable_dict)
    solver2 = commonAlgorithmByStrengthenedGurobi(problem, input_variable)
    time3 = time.time()
    model2 = solver2.run_gurobi_model()
    time4 = time.time()
    print(time4-time3)
    print(obj)
    print(model.objVal)
    print(model2.objVal)
    # test the T variable 
    variable_dict, is_solved = get_variable_from_solved_model(input_variable, ['T'], model2)
    obj, info = solver2.efficient_integrated_evaluate(problem, solution['picking'], solution['sorting'], variable_dict)
    # print(len(info['passTime']))
    # for i in range(len(info['passTime'])):
    #     print(variable_dict['T'][i], info['passTime'][i])