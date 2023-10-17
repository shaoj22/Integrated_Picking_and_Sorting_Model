'''
File: generate_instances.py
Created Date: April 27th 2023
'''

def generate_small_instances():
    wl_list = [[2,4],[3,4],[4,4],[5,4],[6,4]]
    N_list = [2,4,6,8,10,12,14,16,18,20]
    O_list = [2,3,4,5,6]
    R_list = [5,10,15,20,25]
    P_list = [2,4,6,8,10]
    instance = []
    for i in range(len(N_list)):
        for j in range(5):
            single_instance = []
            bins_num = N_list[i]
            w_num = 2
            l_num = wl_list[j][1]
            order_num = O_list[j]
            picking_station_num = P_list[j]
            robot_num = R_list[j]
            single_instance.append(w_num)
            single_instance.append(l_num)
            single_instance.append(bins_num)
            single_instance.append(robot_num)
            single_instance.append(picking_station_num)
            single_instance.append(order_num)
            instance.append(single_instance)
    return instance

def generate_medium_instances():
    wl_list = [[4,6],[5,6],[6,6],[7,6],[8,6]]
    N_list = [10,20,30,40,50,60,70,80,90,100]
    O_list = [10,15,20,25,30]
    R_list = [10,15,20,25,30]
    P_list = [10,10,10,10,10]
    instance = []
    for i in range(len(N_list)):
        for j in range(5):
            single_instance = []
            bins_num = N_list[i]
            w_num = 2
            l_num = wl_list[j][1]
            order_num = O_list[j]
            picking_station_num = P_list[j]
            robot_num = R_list[j]
            single_instance.append(w_num)
            single_instance.append(l_num)
            single_instance.append(bins_num)
            single_instance.append(robot_num)
            single_instance.append(picking_station_num)
            single_instance.append(order_num)
            instance.append(single_instance)
    return instance

def generate_large_instances():
    wl_list = [[6,8],[7,8],[8,8],[9,8],[10,8]]
    N_list = [50,100,150,200,250,300,350,400,450,500]
    O_list = [20,40,60,80,100]
    R_list = [10,20,30,40,50]
    P_list = [10,10,10,10,10]
    instance = []
    for i in range(len(N_list)):
        for j in range(5):
            single_instance = []
            bins_num = N_list[i]
            w_num = 2
            l_num = wl_list[j][1]
            order_num = O_list[j]
            picking_station_num = P_list[j]
            robot_num = R_list[j]
            single_instance.append(w_num)
            single_instance.append(l_num)
            single_instance.append(bins_num)
            single_instance.append(robot_num)
            single_instance.append(picking_station_num)
            single_instance.append(order_num)
            instance.append(single_instance)
    return instance
