'''
File: NNH_heuristic_algorithm for init solution generate
Created Date: April 27th 2023
'''
import Integrated_Instance
import numpy as np
import utils

class NNH_heuristic_algorithm():

    def __init__(self, instance):
        self.n = instance.n
        self.R = instance.robotNum
        self.dis_matrix = instance.disMatrix
        self.capacity = instance.capacity
        self.nodeNum = instance.nodeNum
        self.robotNum = instance.robotNum
    
    # input instances, output solution(chromosome)
    def NNH_main(self):
        # 初始化编码
        car_chromosome = [[] for i in range(self.R)]
        # 初始机器人的位置
        for i in range(self.R):
            if len(car_chromosome[i]) == 0:
                car_chromosome[i].append(4 * self.n + i)
        # 初始化P1和P2的任务
        P1_tasks = [ i for i in range(self.n)]
        P2_tasks = [ i + self.n for i in range(self.n)]
        # 初始化任务分配——全部车都参与完成P1任务
        r = 0 # 车辆编号
        t = 0 # 任务编号
        while P1_tasks:
            # 若车辆超出上限，代表初始化任务分配结束
            if len(car_chromosome[r]) == self.capacity + 1:
                break
            # 计算下一个应该装入的任务
            if len(car_chromosome[r]) == 0:
                next_point = self.choose_nearest_tasks(4 * self.n + r, P1_tasks)
            else:
                # 上一个点到最近的一个点
                next_point = self.choose_nearest_tasks(car_chromosome[r][-1], P1_tasks)
            # 判断能否将任务装入r中，若能则装入，若不能则换成r+1车
            # 装入车
            car_chromosome[r].append(next_point)
            # 去掉装入的点
            P1_tasks.remove(next_point)
            r += 1
            if r == self.R:
                r = 0
        car_chromosome = self.add_destination(car_chromosome)

        # 添加剩余的P1_tasks
        r = 0
        l = 0
        while len(P1_tasks) != 0:
            next_point = self.choose_nearest_tasks(car_chromosome[r][-1], P1_tasks)
            car_chromosome[r].append(next_point)
            P1_tasks.remove(next_point)
            if len(P1_tasks) == 0:
                car_chromosome = self.add_destination(car_chromosome)
                break
            l += 1
            if l == 8:
                car_chromosome = self.add_destination(car_chromosome)
                l = 0
                if r < self.R//2 - 1:
                    r += 1
                else:
                    r = 0

        # 添加所有的P2_tasks
        r = self.R//2
        l = 0
        # 记录之前已经装了多少货
        already_load = []
        for i in range(len(car_chromosome)):
            already_load.append(len(car_chromosome[i]))
        while len(P2_tasks) != 0:
            next_point = self.choose_nearest_tasks(car_chromosome[r][-1], P2_tasks)
            car_chromosome[r].append(next_point)
            P2_tasks.remove(next_point)
            if len(P2_tasks) == 0:
                car_chromosome = self.add_destination(car_chromosome)
                break
            if (len(car_chromosome[r]) - already_load[i]) % self.capacity == 0:
                car_chromosome = self.add_destination(car_chromosome)
            r += 1
            if r == self.R:
                if len(car_chromosome[r-1]) >= (self.n * 4 / self.R) - 10:
                    r = 0
                else:
                    r = self.R//2 
        sum = 0
        for i in range(len(car_chromosome)):
            sum += len(car_chromosome[i])
        return car_chromosome


    # 为分配了任务后的车添加任务的目的地
    def add_destination(self, car_chromosome):
        for i in range(len(car_chromosome)):
            if len(car_chromosome[i]) !=0:
                for j in range(len(car_chromosome[i])):
                    if (car_chromosome[i][j] + self.n * 2) not in car_chromosome[i] and car_chromosome[i][j] < 2 * self.n:
                        car_chromosome[i].append( car_chromosome[i][j] + 2 * self.n)

        return car_chromosome

    # some useful tools for NNH heuristic algorithm
    # input 上一个任务点和当前待完成的任务，output 下一个任务点。
    def choose_nearest_tasks(self, point, tasks):
        min_dis = self.dis_matrix[point][tasks[0]]
        next_point = 0
        for i in range(len(tasks)):
            if self.dis_matrix[point][tasks[i]] <= min_dis:
                min_dis = self.dis_matrix[point][tasks[i]]
                next_point = tasks[i]

        return next_point

    def transfer(self, solution):
        x = np.zeros((self.nodeNum, self.nodeNum))
        for k in range(self.robotNum):
            route = solution[k]
            for i in range(1, len(route)):
                x[route[i-1], route[i]] = 1
            x[route[-1], route[0]] = 1
        return x
    

if __name__ == "__main__":
    # generate instance]
    w_num = 5
    l_num = 5
    bins_num = 15 
    robot_num = 5
    picking_station_num = 2
    orders_num = 5
    instance = Integrated_Instance.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    NNH = NNH_heuristic_algorithm(instance)
    solution = NNH.NNH_main()
    Obj = utils.efficient_picking_evaluate(instance, solution)
    print(Obj)
    Obj2 = utils.picking_integrated_evaluate(instance, NNH.transfer(solution))
    print(Obj2)

    


