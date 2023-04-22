import gurobipy as gp
from gurobipy import GRB
import time
from instance import Instance

class Picking_Gurobi_Model():
    def __init__(self, Instance, time_limit=None):
        # time_limit
        self.time_limit = time_limit
        self.Q = Instance.capacity
        self.delta_T = Instance.pick_time
        # point
        self.N = list(range(Instance.nodeNum))
        self.nodes = Instance.nodes
        self.n = Instance.n
        self.P1 = Instance.P1
        self.P2 = Instance.P2
        self.D1 = Instance.D1
        self.D2 = Instance.D2
        self.W = Instance.W
        # robot
        self.K = list(range(Instance.robotNum))
        # distance and time
        self.disMatrix = Instance.disMatrix
        self.timeMatrix = Instance.timeMatrix
        self.bigM = 300





    def run_gurobi(self):
        M = self.bigM # 大M
        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('Picking_Gurobi_Model') # 创建Gurobi模型
        # MODEL.setParam('OutputFlag', 0)
        # 添加决策变量
        x_list = [(i,j,k) for i in self.N for j in self.N for k in self.K]
        x = MODEL.addVars(x_list, vtype=GRB.BINARY, name="x")  # 车k从点i去点j
        Q_list = [(i,k) for i in self.N for k in self.K]
        Q = MODEL.addVars( Q_list, vtype=GRB.CONTINUOUS, name="Q")  # 车k在i的载重
        T_list = [(i,k) for i in self.N for k in self.K]
        T = MODEL.addVars( T_list, vtype=GRB.CONTINUOUS, name="T")  # 车k在i的时间
        FT = MODEL.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间
        f_list = [(i,j,k) for i in (self.P1 + self.P2) for j in (self.P1 + self.P2) for k in self.K]
        f = MODEL.addVars( f_list, vtype=GRB.BINARY, name="f")
        # 添加优化类型和目标函数
        MODEL.modelSense = GRB.MINIMIZE
        MODEL.setObjective( FT )
        # 添加约束条件
        # 1. 最大完成时间约束：
        MODEL.addConstrs( FT >= T[i,k] for i in self.N for k in self.K)
        # 2. 流平衡约束：
        MODEL.addConstrs( gp.quicksum( x[i,j,k] for j in self.N if j !=i ) == gp.quicksum( x[j,i,k] for j in self.N if j !=i ) for i in self.N for k in self.K )
        # # 3. 完成所有任务：
        MODEL.addConstrs( gp.quicksum( x[i,j,k] for j in self.N for k in self.K if j != i) >= 1 for i in self.N)
        # # 4. 同一个任务用同一个车
        MODEL.addConstrs( gp.quicksum( x[i,j,k] for j in self.N if j !=i) == gp.quicksum( x[j, 2 * self.n + i,k] for j in self.N) for i in (self.P1+self.P2) for k in self.K)
        # 5. 载重约束
        MODEL.addConstrs( gp.quicksum( Q[i,k] for i in self.N ) <= self.Q for k in self.K)
        # MODEL.addConstrs( Q[j,k] >= Q[i,k] + self.nodes[j]["demand"] -  self.Q * (1 - x[i,j,k]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) for k in self.K if i != j)
        # MODEL.addConstrs( Q[i,k] >= 0 for i in self.N for k in self.K)
        # MODEL.addConstrs( Q[i,k] <= self.Q for i in self.N for k in self.K)
        # 6. 时间约束
        MODEL.addConstrs( T[j,k] >= T[i,k] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"] - M * (1 - x[i,j,k]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) for k in self.K)
        # 到达终点的时间=起点+服务+路程时间
        MODEL.addConstrs( T[2 * self.n + i,k] >= T[i,k] + self.timeMatrix[i, 2 * self.n + i] + self.nodes[i]["serviceTime"] for i in (self.P1 + self.P2) for k in self.K)

        MODEL.addConstrs( T[i, k] >= self.nodes[i]["readyTime"] for i in self.N for k in self.K)
        MODEL.addConstrs( T[i, k] <= self.nodes[i]["dueTime"] for i in self.N for k in self.K)
        # 到达P2的时间>=D1的时间+分拣站的服务时间
        MODEL.addConstrs( T[i - self.n,k] >= T[i, k] + self.delta_T for i in self.D1 for k in self.K)

        MODEL.addConstrs( T[i,k] - T[j,k] >= M * (f[i,j,k] - 1) for i in (self.P1 + self.P2) for j in (self.P1 + self.P2) for k in self.K)
        MODEL.addConstrs( T[i,k] - T[j,k] <= M * f[i,j,k] for i in (self.P1 + self.P2) for j in (self.P1 + self.P2) for k in self.K)
        MODEL.addConstrs( T[i + 2 * self.n, k] - T[j + 2 * self.n, k] >= M * (f[i, j,k] - 1) for i in (self.P1 + self.P2) for j in (self.P1 + self.P2) for k in self.K)
        MODEL.addConstrs( T[i + 2 * self.n, k] - T[j + 2 * self.n, k] <= M * f[i, j,k] for i in (self.P1 + self.P2) for j in (self.P1 + self.P2) for k in self.K)












        # --------------------------------------------------------------------------------------------------------------
        # 求解模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        # 非线性优化参数
        MODEL.optimize()
        end_Time = time.time()
        Time = end_Time - start_Time

        # 输出并记录解的情况
        if MODEL.status == 2:
            Obj = MODEL.ObjVal
            # 记录下表是IP的解
            SolutionT = []
            for i in self.N:
                T = []
                for k in self.K:
                    var_name1 = f"T[{i},{k}]"
                    T_i_k = MODEL.getVarByName(var_name1).X
                    T.append(T_i_k)
                SolutionT.append(T)


        else:
            Obj = 0
        objBound = MODEL.objBound
        end_Time = time.time()
        Time =  end_Time - start_Time
        return MODEL, Obj, Time, objBound, SolutionT

if __name__ == "__main__":
    w_num = 2
    l_num = 3
    task_num = 2
    robot_num = 1
    problem = Instance(w_num, l_num, task_num, robot_num)
    alg = Picking_Gurobi_Model(Instance = problem, time_limit = 3600)
    model, Obj, Time, objBound, SolutionT= alg.run_gurobi()
    print("最优解为：", Obj)
    print("上界：",objBound)
    print(Time)
    print(SolutionT)
