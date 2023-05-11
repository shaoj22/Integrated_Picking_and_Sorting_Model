import numpy as np
import random

class Instance():
    def __init__(self, T = 0, N = 5, P = 5,  O = 2 ):
        '''
        :param A: 用于控制料箱到达时间的常量矩阵
        :param S: 料箱i在拣选站处完成拣选需要的时间，即拣选时间；
        :param N: 料箱的总数量；
        :param P: 拣选站的总数量；
        :param dip: 料箱i从入口到拣选站p的距离矩阵；
        :param dpi: 料箱i从拣选站p到出口的距离矩阵；
        :param v: 料箱移动的速度
        :param O: 订单任务数量的总数量；
        :param IO：料箱i是否属于订单任务O；
        :param f1: 前后料箱到达相差的单位时间；
        :param T1 : 单个料箱单位拣选时间；
        :param bigM: 模型中的大M
        '''
        self.N = N
        self.P = P
        self.A = [ i for i in range(1,self.N+1)]
        self.O = O
        self.T1 = 10
        self.bigM = 500
        self.v = 0.25
        self.f1 = 10
        # 生成入口至拣选站的距离矩阵（固定长度为10）
        dip = []
        dpi = []
        x1 = 0
        for i in range(self.P):
            x1 += 5
            dip.append(x1)
            x2 = 55 - x1
            dpi.append(x2)
        Dip = [dip  for _ in range(self.N)]
        # 生成出口至拣选站的距离矩阵
        Dpi = [dpi  for _ in range(self.N)]
        self.Dip = Dip
        self.Dpi = Dpi
        # 初始化料箱i的拣选时间
        self.S = [ self.T1 for _ in range(self.N)]
        # 初始化料箱i与订单o直接的关系（随机初始化）
        self.IO = [[0 for _ in range(self.O)] for _ in range(self.N)]
        for i in range(self.N):
            # 该料箱属于几个订单,n代表最多属于几个订单
            n = random.randint(1,3)
            for j in range(n):
                m = random.randint(0,self.O-1)
                self.IO[i][m] = 1
        # for i in range(self.O):
        #     # 该订单拥有几个料箱，n代表最多拥有几个料箱
        #     n = random.randint(1,3)
        #     for j in range(n):
        #         m = random.randint(0,self.N-1)
        #         self.IO[m][i] = 1
        # 初始化总的料箱任务数量
        sum1 = 0
        for i in range(len(self.IO)):
            sum1 += sum(self.IO[i])
        self.sumIO = sum1

        if T != 0:
            self.I = T
        else:
            self.I = [ i * 10 for i in range(self.N)]


if __name__ == "__main__":
    problem = Instance()
    print(problem.IO)
    print(problem.Dip)
    print(problem.Dpi)
    print(problem.sumIO)
    print(problem.I)