'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-19 19:59:06
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-05-22 11:59:42
FilePath: \Integrated_Picking_and_Sorting_Model\paper2_numerical_experiment\\mdmp_experiment.py
Description: mdmp experiment.
'''


import sys
sys.path.append('..')
import xlwt
import time
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.mp_md_algorithm import MpMdAlgorithm
from Integrated_Picking_and_Sorting_Model.paper2_numerical_experiment.experiment_instances import *


def mdmp_experiment_runner(instances):
    """" input: instances, output: mdmp solution  """
    mdmp_results = []
    book = xlwt.Workbook(encoding='utf-8')
    sheet = book.add_sheet("mdmp")
    save_path = "../Integrated_Picking_and_Sorting_Model/paper2_numerical_experiment/result/mdmp_results_of_large_scale_instances.xls"
    mdmp_idx = 0
    row_idx = 0
    for instance in instances:
        # if mdmp_idx < 2:
        #     mdmp_idx += 1
        #     continue
        each_mdmp_idx = 0
        each_mdmp_upper_result = []
        each_mdmp_lower_result = []
        each_mdmp_time = []
        for i in range(1):
            mdmp_solver = MpMdAlgorithm(instance[i])
            start_time = time.time()
            objective_dict = mdmp_solver.runner()
            end_time = time.time()
            # output the mdmp's result into a file
            cpu_times = end_time - start_time
            upper_bound = objective_dict['upper_bound']
            lower_bound = objective_dict['lower_bound']
            each_mdmp_upper_result.append(upper_bound)
            each_mdmp_lower_result.append(lower_bound)
            each_mdmp_time.append(cpu_times)
        # save the mdmp's result into a excel file
        avg_upper_bound = sum(each_mdmp_upper_result) / len(each_mdmp_upper_result)
        avg_lower_bound = sum(each_mdmp_lower_result) / len(each_mdmp_lower_result)
        avg_time = sum(each_mdmp_time) / len(each_mdmp_time)
        sheet.write(row_idx, 0, mdmp_idx)
        sheet.write(row_idx, 1, avg_upper_bound)
        sheet.write(row_idx, 2, avg_lower_bound)
        sheet.write(row_idx, 3, avg_time)
        each_mdmp_idx += 1
        row_idx += 1
        book.save(save_path)
        mdmp_idx += 1

    return mdmp_results


if __name__ == "__main__":
    # generate instances
    small_scale_instances_matrix = generate_small_scale_instances_matrix()
    medium_scale_instances_matrix = generate_medium_scale_instances_matrix()
    large_scale_instance_matrix = generate_large_scale_instances_matrix()

    # small_scale_instances = generate_instances(small_scale_instances_matrix)
    # medium_scale_instances = generate_instances(medium_scale_instances_matrix)
    large_scale_instances = generate_instances(large_scale_instance_matrix)

    # run instances
    # mdmp_results_1 = mdmp_experiment_runner(small_scale_instances)
    # mdmp_results_2 = mdmp_experiment_runner(medium_scale_instances)
    mdmp_results_3 = mdmp_experiment_runner(large_scale_instances)