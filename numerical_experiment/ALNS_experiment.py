'''
Author: shaoj22 935619774@qq.com
Date: 2023-10-07 18:46:30
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-01-23 09:16:49
FilePath: \Integrated_Picking_and_Sorting_Model\numerical_experiment\ALNS_experiment.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
'''
File: ALNS_experiment.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
do some experiment for the ALNS algorithm.
----------
Author: 626
Created Date: 2023.10.19
'''

import sys
sys.path.append('..')
import Integrated_Gurobi_Model
import Integrated_Instance
import xlwt
import generate_instances
import time 
import Integrated_ALNS


# 产生每个算例
book = xlwt.Workbook(encoding='utf-8')
sheet = book.add_sheet("ALNS_solution")
# 构建excel的表头
sheet.write(0, 0, "算例编号")
sheet.write(0, 1, "W")
sheet.write(0, 2, "L")
sheet.write(0, 3, "N")
sheet.write(0, 4, "R")
sheet.write(0, 5, "P")
sheet.write(0, 6, "O")
sheet.write(0, 7, "Obj")
sheet.write(0, 8, "Time")
# get small instances
instance = generate_instances.generate_large_instances()
# 求解并输出每个算例
for i in range(0, len(instance)):
    ALNS_experiment_instance = Integrated_Instance.Instance(instance[i][0],instance[i][1],instance[i][2],instance[i][3],instance[i][4],instance[i][5])
    alg = Integrated_ALNS.ALNS(ALNS_experiment_instance, iter_num=10000)
    start = time.time()
    solution, obj = alg.run()
    time_cost1 = time.time() - start
    # alg.show_process()
    # print(alg.repair_operators_scores / alg.repair_operators_steps)
    # print("\nbest_obj = {}, time_cost = {}\n\nbest_solution: {}".format(obj, time_cost1, solution))
    # 输出到excel中
    sheet.write(i + 1, 0, i + 1)
    sheet.write(i + 1, 1, instance[i][0])
    sheet.write(i + 1, 2, instance[i][1])
    sheet.write(i + 1, 3, instance[i][2])
    sheet.write(i + 1, 4, instance[i][3])
    sheet.write(i + 1, 5, instance[i][4])
    sheet.write(i + 1, 6, instance[i][5])
    sheet.write(i + 1, 7, obj)
    sheet.write(i + 1, 8, time_cost1)
    # 保存excel文件
    save_path = "C:\\Users\\93561\\Desktop\\code\\Integrated_Picking & Sorting_Model\\output\\ALNS_large_solution.xls"
    book.save(save_path)
