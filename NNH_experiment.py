'''
File: NNH_experiment.py
Created Date: April 25th 2023
'''

import Integrated_Instance
import xlwt
import generate_instances
from NNH_heuristic_algorithm import NNH_heuristic_algorithm as alg
import utils
import time

# 产生每个算例
book = xlwt.Workbook(encoding='utf-8')
sheet = book.add_sheet("gurobi_solution")
# 构建excel的表头
sheet.write(0, 0, "算例编号")
sheet.write(0, 1, "W")
sheet.write(0, 2, "L")
sheet.write(0, 3, "N")
sheet.write(0, 4, "R")
sheet.write(0, 5, "P")
sheet.write(0, 6, "O")
sheet.write(0, 7, "Obj")
sheet.write(0, 8, "Obj2")
sheet.write(0, 9, "Time")
# get small instances
instance = generate_instances.generate_small_instances()
# 求解并输出每个算例
for i in range(0,len(instance)):
    NNH_experiment_instance = Integrated_Instance.Instance(instance[i][0],instance[i][1],instance[i][2],instance[i][3],instance[i][4],instance[i][5])
    NNH = alg(NNH_experiment_instance)
    start_time = time.time()
    solution = NNH.NNH_main()
    end_time = time.time()
    Time = end_time - start_time
    Obj = utils.efficient_picking_evaluate(NNH_experiment_instance, solution)
    Obj2 = utils.picking_integrated_evaluate(NNH_experiment_instance, NNH.transfer(solution))
    # 输出到excel中
    sheet.write(i + 1, 0, i + 1)
    sheet.write(i + 1, 1, instance[i][0])
    sheet.write(i + 1, 2, instance[i][1])
    sheet.write(i + 1, 3, instance[i][2])
    sheet.write(i + 1, 4, instance[i][3])
    sheet.write(i + 1, 5, instance[i][4])
    sheet.write(i + 1, 6, instance[i][5])
    sheet.write(i + 1, 7, Obj)
    sheet.write(i + 1, 8, Obj2)
    sheet.write(i + 1, 9, Time)
    # 保存excel文件
    save_path = "D:\PythonProjet\SF-learning\HaiSystem_Integrated_Model\Integrated_Picking & Sorting_Model\\NNH_solution.xls"
    book.save(save_path)