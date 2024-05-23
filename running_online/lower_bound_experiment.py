'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-19 19:59:06
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-05-22 17:41:00
FilePath: \Integrated_Picking_and_Sorting_Model\paper2_numerical_experiment\lower_bound_experiment.py
Description: lower bound experiment.
'''


import csv
import time
from linear_relaxed_gurobi_model import LinearRelaxedGurobiModel
from experiment_instances import *

def lower_bound_experiment_runner(instances):
    """" input: instances, output: models """
    models = []

    # 设置保存路径，使用相对路径，表示文件将保存在当前文件夹下
    save_path = "./lower_bound_large_scale_instance_results.csv"
    
    # 打开CSV文件并创建写入对象
    with open(save_path, mode='w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
        
        # 写入CSV表头
        writer.writerow(["Model Index", "Average Upper Bound", "Average Lower Bound", "Average Time"])
        
        model_idx = 0
        for instance in instances:
            each_model_upper_result = []
            each_model_lower_result = []
            each_model_time = []
            for i in range(1):
                lower_bound_solver = LinearRelaxedGurobiModel(instance[i])
                start_time = time.time()
                model = lower_bound_solver.run_gurobi_model()
                end_time = time.time()
                
                # 保存模型的结果
                models.append(model)
                cpu_times = end_time - start_time
                upper_bound = model
                lower_bound = model
                each_model_upper_result.append(upper_bound)
                each_model_lower_result.append(lower_bound)
                each_model_time.append(cpu_times)
            
            # 计算平均值并写入CSV文件
            avg_upper_bound = sum(each_model_upper_result) / len(each_model_upper_result)
            avg_lower_bound = sum(each_model_lower_result) / len(each_model_lower_result)
            avg_time = sum(each_model_time) / len(each_model_time)
            writer.writerow([model_idx, avg_upper_bound, avg_lower_bound, avg_time])
            
            model_idx += 1

    return models

if __name__ == "__main__":
    # 生成实例
    small_scale_instances_matrix = generate_small_scale_instances_matrix()
    # medium_scale_instances_matrix = generate_medium_scale_instances_matrix()
    # large_scale_instances_matrix = generate_large_scale_instances_matrix()

    small_scale_instances = generate_instances(small_scale_instances_matrix)
    # medium_scale_instances = generate_instances(medium_scale_instances_matrix)
    # large_scale_instances = generate_instances(large_scale_instances_matrix)

    # 运行实例
    models1 = lower_bound_experiment_runner(small_scale_instances)
    # models2 = lower_bound_experiment_runner(medium_scale_instances)
    # models3 = lower_bound_experiment_runner(large_scale_instances)

