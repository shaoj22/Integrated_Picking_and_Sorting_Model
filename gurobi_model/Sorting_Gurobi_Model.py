'''
File: integrated_gurobi_model.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
complete gurobi model of the Sorting problem.
----------
Author: 626
Created Date: 2023.10.19
'''


import gurobipy as gp
from gurobipy import GRB
import time
from Sorting_Instance import Instance
import numpy as np
import generate_instances
import xlwt


class Sorting_Gurobi():
    '''
    1. 环形输送机分拣优化的Gurobi模型；
    2. 角色：料箱、订单、拣选站；
    3. 料箱属于订单，订单被分配给拣选站，一个料箱可以属于多个订单，但一个订单只能被分配给一个拣选站；
    4. 拣选站有buffer，即缓存区存在大小，不是无限大；
    5. 料箱进入输送机的顺序未知，决策变量待优化；
    6. 优化目标是完成一系列料箱的拣选时间最短；
    '''
    def __init__(self,Instance, time_limit=None):
        '''
        :param Instance.A: 用于控制料箱到达时间的常量矩阵
        :param Instance.S: 料箱i在拣选站处完成拣选需要的时间，即拣选时间；
        :param Instance.N: 料箱的总数量；
        :param Instance.P: 拣选站的总数量；
        :param Instance.v: 料箱移动的速度；
        :param Instance.O: 订单任务的总数量；
        :param Instance.IO: 料箱i是否属于订单o；
        :param Instance.f1: 任意两个连续料箱到达的间隔单位时间；
        :param Instance.time_limit: Gurobi求解时间的上限；
        :param Instance.bigT: 最大时间的上限;
        :param Instance.sumIO: 所有任务的总数量；
        :param Instance.bigM: 模型中的大M；
        '''
        self.A = Instance.A
        self.S = Instance.S
        self.N = Instance.N
        self.P = Instance.P
        self.Dip = Instance.Dip
        self.Dpi = Instance.Dpi
        self.v = Instance.v
        self.O = Instance.O
        self.IO = Instance.IO
        self.f1 = Instance.f1
        self.T1 = Instance.T1
        self.sumIO = Instance.sumIO
        self.bigM = 10000 
        self.I = Instance.I
        self.time_limit = time_limit

    def build_model(self, MODEL):
        M = self.bigM # 大M
        # MODEL.setParam('OutputFlag', 0)
        # 添加决策变量
        y_list = [(i, p) for i in range(self.N) for p in range(self.P)]
        y = MODEL.addVars(y_list, vtype=GRB.BINARY, name="y") # 料箱i是否被分配给了拣选站p
        z_list = [(o, p) for o in range(self.O) for p in range(self.P)]
        z = MODEL.addVars(z_list, vtype=GRB.BINARY, name="z") # 订单o是否被分配给了拣选站p
        Ta_list = [(i, p) for i in range(self.N) for p in range(self.P)]
        Ta = MODEL.addVars(Ta_list, vtype=GRB.INTEGER, name="Ta") # 料箱i到达拣选站p的时间
        Ts_list = [(i, p) for i in range(self.N) for p in range(self.P)]
        Ts = MODEL.addVars(Ts_list, vtype=GRB.INTEGER, name="Ts")  # 料箱i在拣选站p的开始拣选时间
        Te_list = [(i,p) for i in range(self.N) for p in range(self.P)]
        Te = MODEL.addVars(Te_list, vtype=GRB.INTEGER, name="Te" ) # 料箱i在拣选站p的结束拣选时间
        f_list = [(i,j,p) for i in range(self.N) for j in range(self.N) for p in range(self.P)]
        f = MODEL.addVars( f_list, vtype=GRB.BINARY, name="f") # 料箱i是否先于料箱j到达拣选站p
        T = MODEL.addVar( vtype=GRB.INTEGER, name="T") # 所有料箱都到达出口的时间

        # 添加优化类型和目标函数
        MODEL.modelSense = GRB.MINIMIZE
        MODEL.setObjective( T )

        # 约束条件1：T要大于等于每一个料箱i到达出口的时间
        MODEL.addConstrs( T >= Te[i,self.P-1] + (self.Dpi[i][self.P-1]/self.v) for i in range(self.N))
        # 约束条件3：控制决策变量f的两条约束
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] <= (1 - f[i,j,p]) * M for i in range(self.N) for j in range(i+1,self.N) for p in range(self.P))
        MODEL.addConstrs( Ta[i,p] - Ta[j,p] >= -f[i,j,p] * M for i in range(self.N) for j in range(i+1,self.N) for p in range(self.P))
        # 约束条件4：先到达拣选站p的料箱i的结束拣选时间要小于等于后到达拣选站p的料箱j的开始拣选时间
        MODEL.addConstrs( Ts[j,p] - Te[i,p] >= M * (f[i,j,p] + y[i,p] + y[j,p] - 3) for i in range(self.N) for j in range(i+1,self.N) for p in range(self.P))
        MODEL.addConstrs( Ts[i,p] - Te[j,p] >= M * ((1- f[i,j,p]) + y[i,p] + y[j,p] - 3) for i in range(self.N) for j in range(i+1,self.N) for p in range(self.P))
        # 约束条件5：只有订单o被分配给了拣选站p & 料箱i属于订单o，那么料箱i一定能被分配给拣选站p
        MODEL.addConstrs( self.IO[i][o] * z[o,p] <= y[i,p] for i in range(self.N) for p in range(self.P) for o in range(self.O))
        # 约束条件6：所有任务都要被完成
        MODEL.addConstrs( gp.quicksum( z[o,p] for p in range(self.P)) == 1 for o in range(self.O))
        MODEL.addConstr( gp.quicksum( y[i,p] for i in range(self.N) for p in range(self.P)) == self.sumIO)
        # 关于1个料箱可以去多个拣选站的约束
        # --------------------------------------------------------------------------------------------------------------
        # 关于料箱i在拣选站p的结束拣选时间
        # 约束条件1：料箱i的结束拣选时间一定大于等于它的开始拣选时间（当yip=0时）
        MODEL.addConstrs( Te[i,p] == Ts[i,p] + self.T1 * y[i,p] for i in range(self.N) for p in range(self.P))
        # 约束条件3：当料箱i去拣选站p时，Te=Ts+S
        # MODEL.addConstrs( Te[i,p] - self.T1 - Ts[i,p] >= (y[i,p] - 1) * M for i in range(self.N) for p in range(self.P))
        # --------------------------------------------------------------------------------------------------------------
        # 关于料箱i到达拣选站p的时间
        # 约束条件1：料箱i到达第一个拣选站p=0时的时间（初始化）：
        MODEL.addConstrs( Ta[i,0] == self.I[i] + self.Dip[i][0]/self.v for i in range(self.N))
        # 约束条件3：料箱到达下一个拣选站的时间为：在上一个拣选站结束拣选的时间+路程时间（----------标记）约束条件3和约束条件2二选一就可以，到底是大于等于还是等于？
        MODEL.addConstrs( Ta[i,p] == Te[i,p-1] + (self.Dip[i][p] - self.Dip[i][p-1])/self.v for i in range(self.N) for p in range(1,self.P))
        # 约束条件4：开始拣选时间的约束——料箱i在p开始拣选的时间一定大于or等于在上一个拣选站结束拣选的时间+路程时间
        MODEL.addConstrs( Ts[i, p] >= Ta[i, p] for i in range(self.N) for p in range(self.P))
        # MODEL.addConstrs( Ts[i,p] - Te[i,p-1] -(self.Dip[i][p] - self.Dip[i][p-1])/self.v >= 0 for i in range(self.N) for p in range(1,self.P))
        # --------------------------------------------------------------------------------------------------------------
        # 拣选站处的缓存区大小约束
        MODEL.addConstrs( Ts[i,p] - Ta[i,p] <= (8-1) * self.T1 for i in range(self.N) for p in range(self.P))

        # --------------------------------------------------------------------------------------------------------------
         # 求解模型
        MODEL.update()
        info = {
            "y" : y, 
            "z" : z, 
        }
        return info

    

    def run_gurobi(self):

        start_Time = time.time() # 记录模型开始计算时间
        MODEL = gp.Model('Sorting_Model') # 创建Gurobi模型
        self.build_model(MODEL) # 创建模型
        if self.time_limit is not None:
            MODEL.setParam("TimeLimit", self.time_limit)
        # MODEL.setParam('OutputFlag', 0)
        MODEL.optimize()
        end_Time = time.time()
        Time = end_Time - start_Time
        # 记录结果
        result_info = {}
        result_info["timecost"] = end_Time - start_Time

        if MODEL.status == 2:
            Obj = MODEL.ObjVal
            objval = MODEL.objval
            objBound = MODEL.objBound
            Gap = MODEL.MIPGap
            # 决策变量Y
            SolutionY = []
            for i in range(self.N):
                Y = []
                for p in range(self.P):
                    var_name1 = f"y[{i},{p}]"
                    y_i_p = MODEL.getVarByName(var_name1).X
                    Y.append(y_i_p)
                SolutionY.append(Y)

            # 决策变量Z
            SolutionZ = []
            for i in range(self.O):
                Z = []
                for j in range(self.P):
                    var_name2 = f"z[{i},{j}]"
                    z_i_j = MODEL.getVarByName(var_name2).X
                    Z.append(z_i_j)
                SolutionZ.append(Z)

        else:
            raise Exception("model is infeasible")
        result_info["decision_variable_y"] = SolutionY
        result_info["decision_variable_z"] = SolutionZ
        result_info["best_obj"] = MODEL.ObjVal
        result_info["upper_bound"] = MODEL.objBound
        result_info["model"] = MODEL 

        # 记录下表是IP的解
        SolutionT = []
        SolutionTs = []
        SolutionTe = []
        SolutionTa = []
        for i in range(self.N):
            TS = []
            TE = []
            TA = []
            for p in range(self.P):
                var_nameTs = f"Ts[{i},{p}]"
                var_nameTe = f"Te[{i},{p}]"
                var_nameTa = f"Ta[{i},{p}]"
                Ts_i_p = MODEL.getVarByName(var_nameTs).X
                Te_i_p = MODEL.getVarByName(var_nameTe).X
                Ta_i_p = MODEL.getVarByName(var_nameTa).X
                TS.append(Ts_i_p)
                TE.append(Te_i_p)
                TA.append(Ta_i_p)
            SolutionT.append(T)
            SolutionTs.append(TS)
            SolutionTe.append(TE)
            SolutionTa.append(TA)
        
        

        return Obj, Time, objval, objBound, Gap


if __name__ == "__main__":
    T = 0 
    small = generate_instances.generate_medium_instances()
    # 产生每个算例
    book = xlwt.Workbook(encoding='utf-8')
    sheet = book.add_sheet("gurobi_solution")
    # 构建excel的表头
    sheet.write(0, 0, "算例编号")
    sheet.write(0, 1, "W")
    sheet.write(0, 2, "L")
    sheet.write(0, 3, "N")
    sheet.write(0, 4, "R")
    sheet.write(0, 5, "P")
    sheet.write(0, 6, "O")
    sheet.write(0, 7, "Obj")
    sheet.write(0, 8, "objval")
    sheet.write(0, 9, "objBound")
    sheet.write(0, 10, "Gap")
    sheet.write(0, 11, "Time")
    for i in range(0,len(small)):
        problem = Instance(T, small[i][2], small[i][4], small[i][5])
        alg = Sorting_Gurobi(Instance = problem, time_limit=360)
        Obj, Time, objval, objBound, Gap = alg.run_gurobi()
        sheet.write(i + 1, 0, i + 1)
        sheet.write(i + 1, 1, small[i][0])
        sheet.write(i + 1, 2, small[i][1])
        sheet.write(i + 1, 3, small[i][2])
        sheet.write(i + 1, 4, small[i][3])
        sheet.write(i + 1, 5, small[i][4])
        sheet.write(i + 1, 6, small[i][5])
        sheet.write(i + 1, 7, Obj)
        sheet.write(i + 1, 8, objval)
        sheet.write(i + 1, 9, objBound)
        sheet.write(i + 1, 10, Gap)
        sheet.write(i + 1, 11, Time)
        # 保存excel文件
        save_path = "C:\\Users\\93561\\Desktop\\code\\Integrated_Picking & Sorting_Model\\sorting_solution_medium.xls"
        book.save(save_path)
        