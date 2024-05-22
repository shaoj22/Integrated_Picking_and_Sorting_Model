'''
Author: shaoj22 935619774@qq.com
Date: 2024-03-19 20:01:38
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-03-20 09:25:42
FilePath: \Integrated_Picking_and_Sorting_Model\paper2_numerical_experiment\experiment_instances.py
Description: generate instances for numerical experiment.
'''


from Integrated_Instance import Instance


def generate_small_scale_instances_matrix():
    """ generate small scale instances matrix """

    small_scale_instances_matrix = [
        [2, 4, 2, 2, 2, 2],
        [2, 4, 3, 2, 2, 2],
        [2, 4, 4, 2, 2, 2],
        [4, 4, 5, 5, 4, 4],
        [4, 4, 6, 5, 4, 4],
        [4, 4, 7, 5, 4, 4],
        [6, 4, 8, 8, 6, 6],
        [6, 4, 9, 8, 6, 6],
        [6, 4, 10, 8, 6, 6],
    ]
    
    return small_scale_instances_matrix

def generate_medium_scale_instances_matrix():
    """ generate medium scale instances matrix """

    medium_scale_instances_matrix = [
        [6, 8, 12, 10, 6, 4],
        [6, 8, 14, 10, 6, 4],
        [6, 8, 16, 10, 6, 4],
        [8, 8, 18, 15, 8, 6],
        [8, 8, 20, 15, 8, 6],
        [8, 8, 22, 15, 8, 6],
        [10, 8, 24, 20, 10, 8],
        [10, 8, 26, 20, 10, 8],
        [10, 8, 28, 20, 10, 8],
    ]
    
    return medium_scale_instances_matrix

def generate_large_scale_instances_matrix():
    """ generate large scale instances matrix """

    large_scale_instances_matrix = [
        [10, 12, 30, 20, 10, 5],
        [10, 12, 60, 30, 15, 5],
        [10, 12, 90, 40, 20, 5],
        [12, 12, 120, 50, 25, 10],
        [12, 12, 150, 60, 30, 10],
        [12, 12, 180, 70, 35, 10],
        [14, 12, 210, 80, 40, 15],
        [14, 12, 240, 90, 45, 15],
        [14, 12, 270, 100, 50, 15],
    ]
    
    return large_scale_instances_matrix


def generate_RL_instances_matrix():
    """ generate RL instances matrix """

    RL_instances_matrix = [
        [5, 5, 30, 30, 10, 6],
        [6, 6, 40, 40, 15, 7],
        [7, 7, 50, 50, 20, 8],
        [8, 8, 60, 60, 25, 9],
        [9, 9, 70, 70, 30, 11],
    ]
    
    return RL_instances_matrix


def generate_instances(instances_matrix):
    """ input instances matrix, output instances """

    instances = []
    for instance_matrix in instances_matrix:
        each_instance = []
        for i in range(10): # 每类算例的规模为10
            instance = Instance(instance_matrix[0], instance_matrix[1], instance_matrix[2], instance_matrix[3], instance_matrix[4], instance_matrix[5], seed=i)
            each_instance.append(instance)
        instances.append(each_instance)
    
    return instances

