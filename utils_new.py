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
    # some init
    obj = 0
    d1_arrived = [False for _ in range(integrated_instance.n)]
    robot_cur_visit_idx = [1 for _ in range(integrated_instance.robotNum)]
    robot_cur_visit_time = [0 for _ in range(integrated_instance.robotNum)]
    robot_completed = [False for _ in range(integrated_instance.robotNum)]
    T = np.zeros(integrated_instance.nodeNum) # 记录车到达每个点的时间
    # 每个车按访问点位前进
    while robot_completed.count(False)>0:
        # 为每个车前进 one step
        for r in range(integrated_instance.robotNum):
            # base case
            if robot_completed[r]:
                continue
            # base case: 如果idx大于当前robot的路线长度则
            if robot_cur_visit_idx[r] == len(picking_solution[r])-1:
                # 当前车已完成
                robot_completed[r] = True
            cur_node = picking_solution[r][robot_cur_visit_idx[r]] # 当前车访问的当前节点
            # 如果是p2点则需判断是否d1点已经完成，并且更新p2的时间
            if integrated_instance.node2type[cur_node] == 'P2':
                # 判断p1是否完成
                if d1_arrived[cur_node-integrated_instance.n]: # 注意！
                    # 若完成：该点时间=max(robot到达时间, 对应p1到达出口的时间)
                    robot_cur_visit_time[r] += integrated_instance.nodes[picking_solution[r][robot_cur_visit_idx[r]-1]]['serviceTime'] + integrated_instance.timeMatrix[picking_solution[r][robot_cur_visit_idx[r]-1], cur_node]
                    picking_station_list = [] # 该料箱需要去哪几个拣选站
                    # 计算该料箱在环形输送机上的拣选时间
                    for o in range(integrated_instance.O):
                        if integrated_instance.IO[cur_node-integrated_instance.n][o]:
                            # 拣选站的索引
                            if sorting_solution[o] not in picking_station_list:
                                picking_station_list.append(sorting_solution[o])
                    picking_time = len(picking_station_list)*integrated_instance.picking_time
                    # 计算总的dT
                    if cur_node == 35:
                        print('p2')
                    T[cur_node] = max(T[cur_node+integrated_instance.n]+(integrated_instance.conveyor_length/integrated_instance.v + picking_time), robot_cur_visit_time[r])
                    T[cur_node] = max(T[cur_node], integrated_instance.nodes[cur_node]['readyTime'])
                    robot_cur_visit_idx[r] += 1 # 进入下一个点
                    robot_cur_visit_time[r] = T[cur_node] # 更新当前时间
                else:
                    # 若未完成-什么都不用干
                    pass
            else:
                # 如果是p1, d1, d2,直接加上时间并前往下一个点
                robot_cur_visit_time[r] += integrated_instance.nodes[picking_solution[r][robot_cur_visit_idx[r]-1]]['serviceTime'] + integrated_instance.timeMatrix[picking_solution[r][robot_cur_visit_idx[r]-1], cur_node]
                T[cur_node] = max(robot_cur_visit_time[r], integrated_instance.nodes[cur_node]['readyTime'])
                robot_cur_visit_time[r] = T[cur_node] # 更新当前时间
                # TODO:惩罚超时
                # 如果是p1点
                if integrated_instance.node2type[cur_node] == 'D1':
                    d1_arrived[cur_node-2*integrated_instance.n] = True
                robot_cur_visit_idx[r] += 1 # 进入下一个点

    info = {
        "T" : T,
        }
    obj = max(T)

    return obj, info