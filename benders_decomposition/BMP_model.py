import gurobipy as gp
from gurobipy import GRB
import time
from Integrated_Instance import Instance
from Picking_Gurobi_Model import Picking_Gurobi_Model

class BMP_model(Picking_Gurobi_Model):
    def __init__(self, instance, time_limit=None):
        super().__init__(instance, time_limit)
        # conveyors
        self.P = instance.P
        self.Dip = instance.Dip
        self.Dpi = instance.Dpi
        self.v = instance.v
        self.O = instance.O
        self.IO = instance.IO
        self.picking_time = instance.picking_time
        self.queue_length = instance.queue_length
        self.sumIO = instance.sumIO
        self.bigM = 1000

    def build_model(self, MODEL):
        M = self.bigM # 大M
        # 添加决策变量
        x_list = [(i,j) for i in self.N for j in self.N]
        x = MODEL.addVars(x_list, vtype=GRB.BINARY, name="x")  # 车k从点i去点j
        u = [(i,k) for i in self.N for k in self.K]
        u = MODEL.addVars( u, vtype=GRB.BINARY, name="passX")  # k是否经过i
        y_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        y = MODEL.addVars(y_list, vtype=GRB.BINARY, name="y") # 料箱i是否被分配给了拣选站p
        z_list = [(o,p) for o in range(self.O) for p in range(self.P)]
        z = MODEL.addVars(z_list, vtype=GRB.BINARY, name="z") # 订单o是否被分配给了拣选站p
        f_list = [(i,j,p) for i in range(self.n) for j in range(self.n) for p in range(self.P)]
        f = MODEL.addVars( f_list, vtype=GRB.BINARY, name="f") # 料箱i是否先于料箱j到达拣选站p
        T_list = [i for i in self.N]
        T = MODEL.addVars( T_list, vtype=GRB.CONTINUOUS, name="T")  # 车k在i的时间
        FT = MODEL.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间
        # 添加优化类型和目标函数
        MODEL.modelSense = GRB.MINIMIZE
        MODEL.setObjective( FT ) # 最大完成时间目标
        # 添加约束条件
        MODEL.addConstrs( FT >= T[i] for i in self.N)
        MODEL.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i ) == gp.quicksum( x[j,i] for j in self.N if j != i ) for i in self.N)
        MODEL.addConstrs( gp.quicksum( x[i,j] for j in self.N if j != i) >= 1 for i in (self.P1 + self.P2 + self.D1 + self.D2))
        MODEL.addConstrs( (x[i, j] == 1) >> (u[i, k] == u[j, k]) for i in self.N for j in self.N for k in self.K )
        MODEL.addConstrs( u[self.W[k], k] == 1 for k in self.K )
        MODEL.addConstrs( gp.quicksum(u[i, k] for k in self.K) == 1 for i in self.N)
        MODEL.addConstrs( u[i, k] == u[i+2*self.n, k] for i in self.P1+self.P2 for k in self.K)
        MODEL.addConstrs( self.IO[i][o] * z[o,p] <= y[i,p] for i in range(self.n) for p in range(self.P) for o in range(self.O))
        MODEL.addConstrs( gp.quicksum( z[o,p] for p in range(self.P)) == 1 for o in range(self.O))
        MODEL.addConstr( gp.quicksum( y[i,p] for i in range(self.n) for p in range(self.P)) == self.sumIO)

        MODEL.update()
        info = {
            "x" : x, 
            "T" : T, 
            "FT" : FT,
            "f" : f, 
        }

        return info


    def run_gurobi(self):
        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('BMP_model') # 创建Gurobi模型
        self.build_model(MODEL) # 创建模型
        # 求解模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        # MODEL.setParam('OutputFlag', 0)
        MODEL.optimize()
        if MODEL.status != 2:
            raise Exception("model is infeasible")

        SolutionX = []
        for i in self.N:
            X = []
            for j in self.N:
                var_nameX = f"x[{i},{j}]"
                x_i_j = MODEL.getVarByName(var_nameX).X
                X.append(x_i_j)
            SolutionX.append(X)
        SolutionY = []
        for i in range(self.n):
            Y = []
            for j in range(self.P):
                var_nameY = f"y[{i},{j}]"
                y_i_j = MODEL.getVarByName(var_nameY).X
                Y.append(y_i_j)
            SolutionY.append(Y)
        SolutionF = []
        for i in range(self.n):
            F1 = []
            for j in range(self.n):
                F2 = []
                for p in range(self.P):
                    var_nameF = f"f[{i},{j},{p}]"
                    f_i_j_p = MODEL.getVarByName(var_nameF).X
                    F2.append(f_i_j_p)
                F1.append(F2)
            SolutionF.append(F1)

        return SolutionX, SolutionY, SolutionF

if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 10
    robot_num = 10
    picking_station_num = 10
    orders_num = 2
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    alg = BMP_model(instance = problem, time_limit = 180)
    x, y, f = alg.run_gurobi()