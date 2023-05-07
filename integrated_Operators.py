'''
File: integrated_Operators.py
Project: Integrated_Picking---Sorting_Model
File Created: Sunday, 7th May 2023 7:27:16 pm
Author: Charles Lee (lmz22@mails.tsinghua.edu.cn)
'''

import numpy as np

class Operator:
    def __init__(self, instance):
        # 获取算例信息
        self.instance = instance

    def get(self, solution):
        # 获取邻域解
        raise NotImplementedError


