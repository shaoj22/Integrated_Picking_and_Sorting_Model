'''
File: heuristic_algorithm.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
a order batching heuristic algorithm.
----------
Author: 626
Created Date: 2023.10.19
'''

import sys
sys.path.append("..")
import numpy as np
from order_batching_algorithm.generate_random_data import generate_random_data

def get_order_batching_fixed_solution_by_heuristic_algorithm(tote_num, picking_station_num, order_num, io_pair):
    """
    set fixed solution for model

    Args:
        tote_num(int): tote's num;
        picking_station_num(int): picking station num;
        order_num(int): order num;
        io_pair([[]]): tote and order's pair.
    Return:
        y(list)
        z(list)
    """
    y_val = np.zeros((tote_num, picking_station_num))
    z_val = np.zeros((order_num, picking_station_num))
    # 按照顺序一个一个将order分配给picking station
    p = 0
    for o in range(order_num):
        z_val[o, p] = 1
        p += 1
        if p == picking_station_num:
            p = 0
    # 根据order的分配结果将tote分配给picking station
    for o in range(order_num):
        for p in range(picking_station_num):
            if z_val[o,p] == 1: # order o to picking station p
                for i in range(tote_num):
                    # 若i属于o那么i要去p
                    if io_pair[i][o] >= 1:
                        y_val[i,p] = 1
    return y_val, z_val


if __name__ == "__main__":
    # 生成随机的io-pair
    max_sku_types = 3
    max_sku_quantity = 5
    num_orders = 10
    num_sku_types = 5
    io_pair = generate_random_data(max_sku_types, max_sku_quantity, num_orders, num_sku_types)
    # 打印生成的随机订单数据
    print("random data:")
    for i, tote in enumerate(io_pair):
        print(f"tote {i + 1}: {tote}")
    # use heuristic algorithm to get the y and z
    y, z = get_order_batching_fixed_solution_by_heuristic_algorithm(max_sku_quantity, 5, num_orders, io_pair)
    print("tote to picking station:")
    print(y)
    print("order to picking station:")
    print(z)