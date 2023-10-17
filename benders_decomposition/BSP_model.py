import gurobipy as gp
from gurobipy import GRB
import time
from Integrated_Instance import Instance
from Picking_Gurobi_Model import Picking_Gurobi_Model
import BMP_model

class BSP_model(Picking_Gurobi_Model):
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

    def build_model(self, MODEL, x, y, f):
        M = self.bigM # 大M
        # 添加决策变量
        Q_list = [i for i in self.N]
        Q = MODEL.addVars( Q_list, vtype=GRB.CONTINUOUS, name="Q")  # 车k在i的载重
        I_list = [ i for i in range(self.n)]
        I = MODEL.addVars( I_list, vtype=GRB.CONTINUOUS, name="I")  # 料箱i的初始到达输送机的时间
        Ta_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Ta = MODEL.addVars(Ta_list, vtype=GRB.CONTINUOUS, name="Ta") # 料箱i到达拣选站p的时间
        Ts_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        Ts = MODEL.addVars(Ts_list, vtype=GRB.CONTINUOUS, name="Ts")  # 料箱i在拣选站p的开始拣选时间
        Te_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Te = MODEL.addVars(Te_list, vtype=GRB.CONTINUOUS, name="Te" ) # 料箱i在拣选站p的结束拣选时间
        T_list = [i for i in self.N]
        T = MODEL.addVars( T_list, vtype=GRB.CONTINUOUS, name="T")  # 车k在i的时间
        FT = MODEL.addVar( vtype=GRB.CONTINUOUS, name="FT") # 所有任务完成的时间
        # 添加优化类型和目标函数
        MODEL.modelSense = GRB.MINIMIZE
        MODEL.setObjective( FT ) # 最大完成时间目标
        # 添加约束条件
        c1 = MODEL.addConstrs( FT >= T[i] for i in self.N)
        c2 = MODEL.addConstrs( Q[j] >= Q[i] + self.nodes[i]["demand"] for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j and x[i][j] == 1 )
        MODEL.addConstrs( Q[i] >= 0 for i in self.N)
        MODEL.addConstrs( Q[i] <= self.Q for i in self.N)
        MODEL.addConstrs( (T[j] >= T[i] + self.timeMatrix[i][j] + self.nodes[i]["serviceTime"]) for i in self.N for j in (self.P1 + self.P2 + self.D1 + self.D2) if i!=j and x[i][j] == 1)
        MODEL.addConstrs( T[i] >= self.nodes[i]["readyTime"] for i in self.N )
        MODEL.addConstrs( T[i] <= self.nodes[i]["dueTime"] for i in self.N )
        MODEL.addConstrs( T[2 * self.n + i] >= T[i] + self.timeMatrix[i][i+2*self.n] + self.nodes[i]["serviceTime"] for i in (self.P1 + self.P2) )
        MODEL.addConstrs( T[i - self.n] >= Te[i - 2 * self.n, self.P-1] + (self.Dpi[i - 2 * self.n][self.P-1]/self.v) for i in self.D1 )
        MODEL.addConstrs( I[i] == T[i + 2 * self.n] for i in range(self.n) )
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] <= (1 - f[i][j][p]) * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] >= -f[i][j][p] * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Ts[j,p] - Te[i,p] >= M * (f[i][j][p] + y[i][p] + y[i][p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Ts[i,p] - Te[j,p] >= M * ((1- f[i][j][p]) + y[i][p] + y[i][p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Te[i,p] == Ts[i,p] + self.picking_time * y[i][p] for i in range(self.n) for p in range(self.P))
        MODEL.addConstrs( Ts[i,p] <= Ta[i,p] + M * y[i][p] for i in range(self.n) for p in range(self.P))
        MODEL.addConstrs( Ta[i,0] == I[i] + self.Dip[i][0]/self.v for i in range(self.n))
        MODEL.addConstrs( Ta[i,p] == Te[i,p-1] + (self.Dip[i][p] - self.Dip[i][p-1])/self.v for i in range(self.n) for p in range(1,self.P))
        MODEL.addConstrs( Ts[i, p] >= Ta[i, p] for i in range(self.n) for p in range(self.P))
        MODEL.addConstrs( Ts[i,p] - Ta[i,p] <= (self.queue_length-1) * self.picking_time for i in range(self.n) for p in range(self.P))
        
        MODEL.update()
        info = {
            "x" : x, 
            "T" : T, 
            "FT" : FT, 
        }

        return c1, c2


    def run_gurobi(self, x, y, f):
        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('BSP_model') # 创建Gurobi模型
        c1, c2 = self.build_model(MODEL, x, y, f) # 创建模型
        # 求解模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        # MODEL.setParam('OutputFlag', 0)
        MODEL.optimize()
        if MODEL.status != 2:
            print(1)
        dual_ray_c1 = MODEL.getAttr('Ray', c1)
        dual_ray_c2 = MODEL.getAttr('Ray', c2)
        #dual_vertex_c1 = MODEL.getAttr('Vertex', c1)
        #dual_vertex_c2 = MODEL.getAttr('Vertex', c2)

        return dual_ray_c1, dual_ray_c2



if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 10
    robot_num = 10
    picking_station_num = 10
    orders_num = 2
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    alg1 = BMP_model.BMP_model(instance = problem, time_limit = 180)
    x, y, f = alg1.run_gurobi()
    alg2 = BSP_model(instance = problem, time_limit = 180)
    ray_c1, ray_c2 = alg2.run_gurobi(x, y, f)

    print(ray_c1)