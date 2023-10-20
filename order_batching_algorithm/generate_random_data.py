'''
File: generate_batching_algorithm.py
Project: Integrated_Picking & Sorting_Model
Description: Stochastic arithmetic for generating orders and skus data
File Created: 2023.10.06
Author: 626
'''


import random
import numpy as np


# 生成随机数据，max_sku_types为订单最大sku类型数，max_sku_quantity为最大sku数量，num_orders为订单数，num_sku_types为sku类型数
def generate_random_data(max_sku_types, max_sku_quantity, num_orders, num_sku_types):
    data = [[0] * num_orders for _ in range(num_sku_types)]  # 初始化矩阵，所有元素为0
    for order_index in range(num_orders):
        # 随机生成订单中的SKU种类数量（最小为1，最大为max_sku_types）
        num_sku_types_in_order = random.randint(1, max_sku_types)
        # 随机选择SKU种类（从0到num_sku_types - 1）
        sku_types_in_order = random.sample(range(num_sku_types), num_sku_types_in_order)
        # 随机生成每个SKU的数量（最小为1，最大为max_sku_quantity）
        sku_quantities = [random.randint(1, max_sku_quantity) for _ in range(num_sku_types_in_order)]
        # 更新矩阵，将订单中存在的SKU数量赋值
        for sku_type, quantity in zip(sku_types_in_order, sku_quantities):
            data[sku_type][order_index] = quantity
            
    return data


if __name__ == "__main__":
    # 示例用法：生成10个订单，每个订单最多包含3种SKU，每种SKU数量最多为5
    max_sku_types = 3
    max_sku_quantity = 5
    num_orders = 10
    num_sku_types = 5
    random_orders = generate_random_data(max_sku_types, max_sku_quantity, num_orders, num_sku_types)
    # 打印生成的随机订单数据
    print("random data:")
    print(random_orders)