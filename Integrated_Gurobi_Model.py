import gurobipy as gp
from gurobipy import GRB
import time
from Integrated_Instance import Instance
from Picking_Gurobi_Model import Picking_Gurobi_Model

class Integrated_Gurobi_Model(Picking_Gurobi_Model):
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
        info = super().build_model(MODEL, delta_T=0) # 构建picking模型
        T = info["T"] # 获取T变量
        FT = info["FT"] # 获取FT变量
        # 添加决策变量
        I_list = [ i for i in range(self.n)]
        I = MODEL.addVars( I_list, vtype=GRB.INTEGER, name="I")  # 料箱i的初始到达输送机的时间
        y_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        y = MODEL.addVars(y_list, vtype=GRB.BINARY, name="y") # 料箱i是否被分配给了拣选站p
        z_list = [(o,p) for o in range(self.O) for p in range(self.P)]
        z = MODEL.addVars(z_list, vtype=GRB.BINARY, name="z") # 订单o是否被分配给了拣选站p
        Ta_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Ta = MODEL.addVars(Ta_list, vtype=GRB.INTEGER, name="Ta") # 料箱i到达拣选站p的时间
        Ts_list = [(i, p) for i in range(self.n) for p in range(self.P)]
        Ts = MODEL.addVars(Ts_list, vtype=GRB.INTEGER, name="Ts")  # 料箱i在拣选站p的开始拣选时间
        Te_list = [(i,p) for i in range(self.n) for p in range(self.P)]
        Te = MODEL.addVars(Te_list, vtype=GRB.INTEGER, name="Te" ) # 料箱i在拣选站p的结束拣选时间
        f2_list = [(i,j,p) for i in range(self.n) for j in range(self.n) for p in range(self.P)]
        f2 = MODEL.addVars( f2_list, vtype=GRB.BINARY, name="f2") # 料箱i是否先于料箱j到达拣选站p


        # 添加约束条件
        # ______________________________________________________________________________________________________________
        # 到达P2的时间>=到达环形输送机出口的时间
        MODEL.addConstrs( T[i - self.n] == Te[i - 2 * self.n, self.P-1] + (self.Dpi[i - 2 * self.n][self.P-1]/self.v) for i in self.D1 )
        # 到达输送机的时间要>=到达D1的时间
        MODEL.addConstrs( I[i] == T[i + 2 * self.n] for i in range(self.n) )
        # ______________________________________________________________________________________________________________


        # 添加约束条件
        # 约束条件3：控制决策变量f的两条约束
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] <= (1 - f2[i,j,p]) * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] >= -f2[i,j,p] * M for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 约束条件4：先到达拣选站p的料箱i的结束拣选时间要小于等于后到达拣选站p的料箱j的开始拣选时间
        MODEL.addConstrs( Ts[j,p] - Te[i,p] >= M * (f2[i,j,p] + y[i,p] + y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        MODEL.addConstrs( Ts[i,p] - Te[j,p] >= M * ((1- f2[i,j,p]) + y[i,p] + y[j,p] - 3) for i in range(self.n) for j in range(i+1,self.n) for p in range(self.P))
        # 约束条件5：只有订单o被分配给了拣选站p & 料箱i属于订单o，那么料箱i一定能被分配给拣选站p
        MODEL.addConstrs( self.IO[i][o] * z[o,p] <= y[i,p] for i in range(self.n) for p in range(self.P) for o in range(self.O))
        # 约束条件6：所有任务都要被完成
        MODEL.addConstrs( gp.quicksum( z[o,p] for p in range(self.P)) == 1 for o in range(self.O))
        MODEL.addConstr( gp.quicksum( y[i,p] for i in range(self.n) for p in range(self.P)) == self.sumIO)
        # 关于1个料箱可以去多个拣选站的约束
        # --------------------------------------------------------------------------------------------------------------
        # 关于料箱i在拣选站p的结束拣选时间
        # 约束条件1：料箱i的结束拣选时间一定大于等于它的开始拣选时间（当yip=0时）
        MODEL.addConstrs( Te[i,p] == Ts[i,p] + self.picking_time * y[i,p] for i in range(self.n) for p in range(self.P))
        # 约束条件3：当料箱i不去拣选站p时，Ts=Ta
        MODEL.addConstrs( Ts[i,p] <= Ta[i,p] + M * y[i,p] for i in range(self.n) for p in range(self.P))
        
        # --------------------------------------------------------------------------------------------------------------
        # 关于料箱i到达拣选站p的时间
        # 约束条件1：料箱i到达第一个拣选站p=0时的时间（初始化）：
        MODEL.addConstrs( Ta[i,0] == I[i] + self.Dip[i][0]/self.v for i in range(self.n))
        # 约束条件3：料箱到达下一个拣选站的时间为：在上一个拣选站结束拣选的时间+路程时间（----------标记）约束条件3和约束条件2二选一就可以，到底是大于等于还是等于？
        MODEL.addConstrs( Ta[i,p] == Te[i,p-1] + (self.Dip[i][p] - self.Dip[i][p-1])/self.v for i in range(self.n) for p in range(1,self.P))
        # 约束条件4：开始拣选时间的约束——料箱i在p开始拣选的时间一定大于or等于在上一个拣选站结束拣选的时间+路程时间
        # MODEL.addConstrs( Ts[i,0] >= Ta[i,0] for i in range(self.n))
        MODEL.addConstrs( Ts[i, p] >= Ta[i, p] for i in range(self.n) for p in range(self.P))
        # MODEL.addConstrs( Ts[i,p] - Te[i,p-1] -(self.Dip[i][p] - self.Dip[i][p-1])/self.v >= 0 for i in range(self.N) for p in range(1,self.P))
        # --------------------------------------------------------------------------------------------------------------
        # 拣选站处的缓存区大小约束
        MODEL.addConstrs( Ts[i,p] - Ta[i,p] <= (self.queue_length-1) * self.picking_time for i in range(self.n) for p in range(self.P))

        info["y"] = y
        info["z"] = z
        return info

    def run_gurobi(self):
        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('Picking_and_Sorting_Gurobi_Model') # 创建Gurobi模型
        self.build_model(MODEL) # 构建模型目标、约束

        # 求解模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        MODEL.setParam("OutputFlag", 1)
        # 非线性优化参数
        MODEL.optimize()
        end_Time = time.time()
        Time = end_Time - start_Time

        # 输出并记录解的情况
        if MODEL.status == 2:
            Obj = MODEL.ObjVal
            # # 记录下表是IP的解
            # SolutionT = []
            # # SolutionTS = []
            # SolutionTo = []
            # # SolutionZ = []
            # # SolutionTe = []
            # # SolutionTa = []
            # for i in self.N:
            #     T = []
            # #     TS = []

            # #     TE = []
            # #     TA = []
            #     for k in self.K:
            #         var_name1 = f"T[{i},{k}]"
            # #         var_name2 = f"Ts[{i},{j}]"
            # #         var_nameTe = f"Te[{i},{j}]"
            # #         var_nameTa = f"Ta[{i},{j}]"
            #         T_i_j = MODEL.getVarByName(var_name1).X
            # #         Ts_i_j = MODEL.getVarByName(var_name2).X
            # #         Te_i_j = MODEL.getVarByName(var_nameTe).X
            # #         Ta_i_j = MODEL.getVarByName(var_nameTa).X
            #         T.append(T_i_j)
            # #         TS.append(Ts_i_j)
            # #         To.append(Te_i_j+self.Dpi[i][j]/self.v)
            # #         TE.append(Te_i_j)
            # #         TA.append(Ta_i_j)
            #     SolutionT.append(T)
            # #     SolutionTS.append(TS)
            # #     SolutionTo.append(To)
            # #     SolutionTe.append(TE)
            # #     SolutionTa.append(TA)
            # # # 记录下表是OP的解
            # # SolutionZ = []
            # # for i in range(self.O):
            # #     Z = []
            # #     for j in range(self.P):
            # #         var_name3 = f"z[{i},{j}]"
            # #         z_i_j = MODEL.getVarByName(var_name3).X
            # #         Z.append(z_i_j)
            # #     SolutionZ.append(Z)
            # # # 记录I的解
            # SolutionI = []
            # for i in range(self.n):
            #     var_name4 = f"I[{i}]"
            #     I_i =MODEL.getVarByName(var_name4).X
            #     SolutionI.append(I_i)
            # for i in range(self.n):
            #     To =[]
            #     for p in range(self.P):
            #         var_nameTe = f"Te[{i},{p}]"
            #         Te_i_p = MODEL.getVarByName(var_nameTe).X
            #         To.append(Te_i_p+self.Dpi[i][p]/self.v)
            #     SolutionTo.append(To)
        else:
            Obj = 0
        objval = MODEL.objval
        objBound = MODEL.objBound
        Gap = MODEL.MIPGap
        end_Time = time.time()
        Time =  end_Time - start_Time
        return MODEL, Obj, Time, objval, objBound, Gap

if __name__ == "__main__":
    w_num = 5
    l_num = 5
    bins_num = 5
    robot_num = 3
    picking_station_num = 10
    orders_num = 2
    problem = Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    alg = Integrated_Gurobi_Model(instance = problem, time_limit = 1800)
    model, Obj, Time, objval, objBound, Gap = alg.run_gurobi()
    problem.render(model=model)
    print("最优解为：", Obj)
    print("上界：",objBound)
    print("下界：",objval)
    print("Gap:",Gap)
    print(Time)