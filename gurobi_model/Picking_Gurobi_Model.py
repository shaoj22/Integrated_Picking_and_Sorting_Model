import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time
import utils
from generate_instances import Picking_Instance
from heuristic_algorithm import NNH_heuristic_algorithm


class Picking_Gurobi_Model():
    def __init__(self, instance, time_limit=None, init_flag=True):
        # time_limit
        self.picking_instance = instance
        self.time_limit = time_limit
        self.init_flag = init_flag
        self.Q = instance.capacity
        self.delta_T = instance.pick_time
        # point
        self.N = list(range(instance.nodeNum))
        self.nodes = instance.nodes
        self.n = instance.n
        self.P1 = instance.P1
        self.P2 = instance.P2
        self.D1 = instance.D1
        self.D2 = instance.D2
        self.W = instance.W
        # robot
        self.K = list(range(instance.robotNum))
        # distance and time
        self.disMatrix = instance.disMatrix
        self.timeMatrix = instance.timeMatrix
    
    def set_init_solution(self, MODEL, strategy = 0):
        """
        set init solution for model
        """
        x_val = np.zeros((self.picking_instance.nodeNum, self.picking_instance.nodeNum))
        # init strategy 1
        init_alg = NNH_heuristic_algorithm.NNH_heuristic_algorithm(self.picking_instance)
        routes = init_alg.NNH_main()
        # init strategy 2
        # set init solution
        for route in routes:
            for i in range(1, len(route)):
                pi = route[i-1]
                pj = route[i]
                x_val[pi, pj] = 1
            x_val[route[-1], route[0]] = 1
        for i in range(self.picking_instance.nodeNum):
            for j in range(self.picking_instance.nodeNum):
                MODEL.getVarByName("x[{},{}]".format(i,j)).start = x_val[i,j]
        MODEL.update()

    def build_model(self, MODEL, delta_T=-1):
        """build objective, variables and constraints in picking problem    

        Args:
            MODEL (gurobi.model): original model
            delta_T (int, optional): picking average time, -1 means use default pick_time. Defaults to -1.
        """
        if delta_T == -1:
            delta_T = self.delta_T
        # 添加决策变量
        x_list = [(i,j) for i in self.N for j in self.N]
        x = MODEL.addVars(x_list, vtype=GRB.BINARY, name="x")  # 车k从点i去点j
        Q_list = [i for i in self.N]
        Q = MODEL.addVars( Q_list, vtype=GRB.CONTINUOUS, name="Q")  # 车k在i的载重
        T_list = [i for i in self.N]
        T = MODEL.addVars( T_list, vtype=GRB.CONTINUOUS, name="T")  # 车k在i的时间
        FT = MODEL.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间
        pass_list = [(i,k) for i in self.N for k in self.K]
        passX = MODEL.addVars( pass_list, vtype=GRB.BINARY, name="passX")  # k是否经过i
        # 添加优化类型和目标函数
        MODEL.modelSense = GRB.MINIMIZE
        MODEL.setObjective( FT ) # 最大完成时间目标
        # MODEL.setObjective( FT + 1e-3*gp.quicksum(T[i] for i in self.N) ) # 最大完成时间 + 总完成时间目标
        # MODEL.setObjective( gp.quicksum(T[i] for i in self.N) ) # 总完成时间目标
        # MODEL.setObjective( gp.quicksum(x[i,j,k] * self.disMatrix[i,j] for i in self.N for j in self.N if i!=j for k in self.K) ) # 总距离目标
        # 添加约束条件
        # 0. 最大完成时间约束
        MODEL.addConstrs( FT >= T[i] for i in self.N)
        # 1. 流平衡约束
        MODEL.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i ) == gp.quicksum( x[j,i] for j in self.N if j != i ) for i in self.N)
        # 2. 完成所有任务
        MODEL.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i) >= 1 for i in (self.P1 + self.P2 + self.D1 + self.D2))
        # 3. 同一个任务用同一个车
        MODEL.addConstrs( (x[i, j] == 1) >> (passX[i, k] == passX[j, k]) for i in self.N for j in self.N for k in self.K )
        MODEL.addConstrs( passX[self.W[k], k] == 1 for k in self.K )
        MODEL.addConstrs( gp.quicksum(passX[i, k] for k in self.K) == 1 for i in self.N)
        MODEL.addConstrs( passX[i, k] == passX[i+2*self.n, k] for i in self.P1+self.P2 for k in self.K)
        # 4. 一个车只能从自己的出发点出发一次
        MODEL.addConstrs( gp.quicksum( x[self.W[k],j] for j in self.N if j != self.W[k]) <= 1 for k in self.K)
        # 5. 载重约束
        MODEL.addConstrs( (x[i, j] == 1) >> (Q[j] >= Q[i] + self.nodes[i]["demand"]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        MODEL.addConstrs( Q[i] >= 0 for i in self.N)
        MODEL.addConstrs( Q[i] <= self.Q for i in self.N)
        # 6. 时间约束
        MODEL.addConstrs( (x[i, j] == 1) >> (T[j] >= T[i] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j)
        MODEL.addConstrs( T[i] >= self.nodes[i]["readyTime"] for i in self.N )
        MODEL.addConstrs( T[i] <= self.nodes[i]["dueTime"] for i in self.N )
        ## 到达终点的时间>=起点+服务+路程时间
        MODEL.addConstrs( T[2 * self.n + i] >= T[i] + self.timeMatrix[i][i+2*self.n] + self.nodes[i]["serviceTime"] for i in (self.P1 + self.P2) )
        ## 到达P2的时间>=D1的时间+分拣站的服务时间
        MODEL.addConstrs( T[i] >= T[i + self.n] + delta_T for i in self.P2 )

        MODEL.update()
        info = {
            "x" : x, 
            "T" : T, 
            "FT" : FT, 
        }
        return info

    def run_gurobi(self):
        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('Picking_Gurobi_Model') # 创建Gurobi模型
        self.build_model(MODEL, delta_T=self.delta_T) # 创建模型
        if self.init_flag:
            self.set_init_solution(MODEL) # 设置初始解
        # 求解模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        # MODEL.setParam('OutputFlag', 0)
        MODEL.optimize()
        if MODEL.status != 2:
            raise Exception("model is infeasible")
        # 记录结果
        result_info = {}
        end_Time = time.time()
        result_info["timecost"] = end_Time - start_Time
        SolutionT = [MODEL.getVarByName(f"T[{i}]").X for i in self.N]
        SolutionX = utils.model2instance_routes(MODEL, self.picking_instance)
        result_info["pass_times"] = SolutionT
        result_info["routes"] = SolutionX
        result_info["best_obj"] = MODEL.ObjVal
        result_info["upper_bound"] = MODEL.objBound
        result_info["model"] = MODEL #? space cost
        return result_info

if __name__ == "__main__":
    w_num = 2
    l_num = 2
    task_num = 20
    robot_num = 10
    instance = Picking_Instance.Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_Gurobi_Model(instance = instance, time_limit = 3600, init_flag = True)
    result_info = alg.run_gurobi()
    instance.render(model=result_info["model"])
    print("最优解为：", result_info["best_obj"])
    print("上界：",result_info["upper_bound"])
    print("用时：", result_info["timecost"])
    print("路径：", result_info["routes"])
