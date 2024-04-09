'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-10 16:33:10
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-03-20 21:44:06
FilePath: /Integrated_Picking_and_Sorting_Model/utils_new.py.
Description: new utils for the fitness evaluate.
'''


import sys
sys.path.append("..")
import numpy as np


def efficient_integrated_evaluate(integrated_instance, picking_solution, sorting_solution):
    """ input instance and picking, sorting solution to get the obj

    Args:
        integrated_instance (class): the instance of the problem.
        picking_solution (List[List[int]]): the routes of the problem.
        sorting_solution (List[int]): the solution of the order to station.

    Returns:
        obj (int): the solution's obj.
        info ({}): some info of the solution. 
    """
    obj = 0
    info = {}
    # 所以任务是否完成
    task_num = 0 # 所有任务计数
    for r in range(len(picking_solution)):
        task_num += len(picking_solution[r]) - 1
    if task_num != integrated_instance.n*4:
        obj += 10000
        return obj, info
    # check the right or not of the input solution.
    # for r in range(integrated_instance.robotNum):
    #     for ri in range(1, picking_solution[r]):
    #         if integrated_instance.node2type[picking_solution[r][ri]] == 'P2':
    # some init
    d1_arrived = [False for _ in range(integrated_instance.n)]
    robot_cur_visit_idx = [1 for _ in range(integrated_instance.robotNum)]
    robot_cur_visit_time = [0 for _ in range(integrated_instance.robotNum)]
    robot_completed = [False for _ in range(integrated_instance.robotNum)]
    robot_need_visit_first = [-1 for _ in range(integrated_instance.robotNum)] # robot需要先完成的robotIdx
    T = np.zeros(integrated_instance.nodeNum) # 记录车到达每个点的时间
    num = 0
    # 每个车按访问点位前进
    while robot_completed.count(False)>0:
        # base case: 不符合约束的解直接退出
        num += 1
        if num == 30000:
            print("error")
            obj += 10000
            return obj, info
        # 为每个车前进 one step
        for r in range(integrated_instance.robotNum):
            # base case1:如果该车未用直接设置完成
            if len(picking_solution[r]) == 1:
                robot_completed[r] = True
            # base case2:
            if robot_completed[r]:
                continue
            # base case3: 如果idx大于当前robot的路线长度则退出
            if robot_cur_visit_idx[r] == len(picking_solution[r])-1:
                # 当前车已完成
                robot_completed[r] = True
            cur_node = picking_solution[r][robot_cur_visit_idx[r]] # 当前车访问的当前节点
            # 如果是p2点则需判断是否d1点已经完成，并且更新p2的时间
            if integrated_instance.node2type[cur_node] == 'P2':
                # P2是否在P1之前
                if (cur_node-integrated_instance.n) in picking_solution[r] \
                    and (picking_solution[r].index(cur_node) < picking_solution[r].index((cur_node-integrated_instance.n)) \
                         or picking_solution[r].index(cur_node) < picking_solution[r].index((cur_node+integrated_instance.n))):
                    obj += 10000
                    return obj, info
                # 判断d1是否完成
                if d1_arrived[cur_node-integrated_instance.n]: # 注意！
                    # 若完成：该点时间=max(robot到达时间, 对应p1到达出口的时间)
                    robot_cur_visit_time[r] += integrated_instance.nodes[picking_solution[r][robot_cur_visit_idx[r]-1]]['serviceTime'] \
                        + integrated_instance.timeMatrix[picking_solution[r][robot_cur_visit_idx[r]-1], cur_node]
                    picking_station_list = [] # 该料箱需要去哪几个拣选站
                    # 计算该料箱在环形输送机上的拣选时间
                    for o in range(integrated_instance.O):
                        if integrated_instance.IO[cur_node-integrated_instance.n][o]:
                            # 拣选站的索引
                            if sorting_solution[o] not in picking_station_list:
                                picking_station_list.append(sorting_solution[o])
                    # TODO: 拣选站真实排队时间计算
                    
                    picking_time = len(picking_station_list)*integrated_instance.picking_time
                    # 计算总的dT
                    T[cur_node] = max(T[cur_node+integrated_instance.n] + \
                                      (integrated_instance.conveyor_length/integrated_instance.v + picking_time), robot_cur_visit_time[r])
                    T[cur_node] = max(T[cur_node], integrated_instance.nodes[cur_node]['readyTime'])
                    robot_cur_visit_idx[r] += 1 # 进入下一个点
                    robot_cur_visit_time[r] = T[cur_node] # 更新当前时间
                else:
                    # 若未完成-记录需要先完成的robotIdx
                    need_visit_first_node = cur_node+integrated_instance.n
                    # 找到need node所在的robot
                    for first_r in range(integrated_instance.robotNum):
                        if need_visit_first_node in picking_solution[first_r]:
                            robot_need_visit_first[r] = first_r
                            # 如果出现cycle则退出
                            if robot_need_visit_first[first_r] == r:
                                obj += 10000
                                return obj, info
            else:
                # 如果是p1, d1, d2,直接加上时间并前往下一个点
                robot_cur_visit_time[r] += integrated_instance.nodes[picking_solution[r][robot_cur_visit_idx[r]-1]]['serviceTime']\
                      + integrated_instance.timeMatrix[picking_solution[r][robot_cur_visit_idx[r]-1], cur_node]
                T[cur_node] = max(robot_cur_visit_time[r], integrated_instance.nodes[cur_node]['readyTime'])
                robot_cur_visit_time[r] = T[cur_node] # 更新当前时间
                # TODO: 惩罚超时计算

                # 如果是d1点
                if integrated_instance.node2type[cur_node] == 'D1':
                    d1_arrived[cur_node-2*integrated_instance.n] = True
                robot_cur_visit_idx[r] += 1 # 进入下一个点

    # record the solution
    info['T'] = T
    obj = max(max(T), obj)

    return obj, info