'''
File: NNH_heuristic_algorithm for init solution generate
Created Date: April 27th 2023
'''
import Integrated_Instance
import small_instances

class NNH_heuristic_algorithm():

    def __init__(self, instance):
        self.n = instance.n
        self.R = instance.robotNum
        self.dis_matrix = instance.disMatrix
        self.capacity = instance.capacity
    
    # input instances, output solution(chromosome)
    def NNH_main(self):
        # 初始化编码
        car_chromosome = [[] for i in range(self.R)]
        # 初始化P1和P2的任务
        P1_tasks = [ i for i in range(self.n)]
        P2_tasks = [ i + self.n for i in range(self.n)]
        # 初始化任务分配——全部车都参与完成P1任务
        r = 0 # 车辆编号
        t = 0 # 任务编号
        while P1_tasks:
            # 若车辆超出上限，代表初始化任务分配结束
            if r >= self.R:
                break
            # 计算下一个应该装入的任务
            if len(car_chromosome[r]) == 0:
                # 起点到最近的一个点
                car_chromosome[r].append(4 * self.n + r)
                next_point = self.choose_nearest_tasks(4 * self.n + r, P1_tasks)
            else:
                # 上一个点到最近的一个点
                next_point = self.choose_nearest_tasks(car_chromosome[r][-1], P1_tasks)
            # 判断能否将任务装入r中，若能则装入，若不能则换成r+1车
            if len(car_chromosome[r]) < self.capacity:
                # 装入车
                car_chromosome[r].append(next_point)
                # 去掉装入的点
                P1_tasks.remove(next_point)
            else:
                r = r + 1
            print(car_chromosome)
        car_chromosome = self.add_destination(car_chromosome)
        print(P1_tasks)
        print(P2_tasks)

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
        while len(P2_tasks) != 0:
            next_point = self.choose_nearest_tasks(car_chromosome[r][-1], P2_tasks)
            car_chromosome[r].append(next_point)
            P2_tasks.remove(next_point)
            if len(P2_tasks) == 0:
                car_chromosome = self.add_destination(car_chromosome)
                break
            l += 1
            if l == 8:
                car_chromosome = self.add_destination(car_chromosome)
                l = 0
                if r < self.R - 1:
                    r += 1
                else:
                    r = self.R//2

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



if __name__ == "__main__":
    # generate instance]
    w_num = 4
    l_num = 4
    bins_num = 1000
    robot_num = 7
    picking_station_num = 2
    orders_num = 2
    instance = Integrated_Instance.Instance(w_num, l_num, bins_num, robot_num, picking_station_num, orders_num)
    NNH = NNH_heuristic_algorithm(instance)
    car_chromosome = NNH.NNH_main()
    sum = 0
    for i in range(len(car_chromosome)):
        sum += len(car_chromosome[i])
        print(len(car_chromosome[i]))
    print(sum)
    


