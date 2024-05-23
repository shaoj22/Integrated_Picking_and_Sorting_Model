'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-19 19:59:06
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-05-08 11:12:43
FilePath: \Integrated_Picking_and_Sorting_Model\paper2_numerical_experiment\\rma_experiment.py
Description: rma experiment.
'''


import sys
sys.path.append('..')
import xlwt
import time
from Integrated_Picking_and_Sorting_Model.two_layer_revolving_algorithm import rma_main
from Integrated_Picking_and_Sorting_Model.paper2_numerical_experiment.experiment_instances import *


def rma_experiment_runner(instances):
    """" input: instances, output: rma solution  """
    rma_results = []
    book = xlwt.Workbook(encoding='utf-8')
    sheet = book.add_sheet("rma")
    save_path = "../Integrated_Picking_and_Sorting_Model/paper2_numerical_experiment/result/rma_results_of_large_scale_instances.xls"
    rma_idx = 0
    row_idx = 0
    for instance in instances:
        if rma_idx <= 7:
            rma_idx += 1
            continue
        each_rma_idx = 0
        each_rma_results = []
        each_rma_time = []
        for i in range(2,3):
            start_time = time.time()
            solution, obj = rma_main.rma_main(instance[i])
            end_time = time.time()
            rma_results.append(obj)
            # output the model's result into a file
            cpu_times = end_time - start_time
            each_rma_results.append(obj)
            each_rma_time.append(cpu_times)
        # save the rma's result into a excel file
        avg_obj = sum(each_rma_results) / len(each_rma_results)
        avg_cpu_times = sum(each_rma_time) / len(each_rma_time)
        sheet.write(row_idx, 0, rma_idx)
        sheet.write(row_idx, 1, avg_obj)
        sheet.write(row_idx, 2, avg_cpu_times)
        each_rma_idx += 1
        row_idx += 1
        book.save(save_path)
        rma_idx += 1

    return rma_results


if __name__ == "__main__":
    # generate instances
    small_scale_instances_matrix = generate_small_scale_instances_matrix()
    medium_scale_instances_matrix = generate_medium_scale_instances_matrix()
    large_scale_instance_matrix = generate_large_scale_instances_matrix()

    # small_scale_instances = generate_instances(small_scale_instances_matrix)
    # medium_scale_instances = generate_instances(medium_scale_instances_matrix)
    large_scale_instances = generate_instances(large_scale_instance_matrix)
    # run instances
    # rma_results_1 = rma_experiment_runner(small_scale_instances)
    # rma_results_2 = rma_experiment_runner(medium_scale_instances)
    rma_results_3 = rma_experiment_runner(large_scale_instances)