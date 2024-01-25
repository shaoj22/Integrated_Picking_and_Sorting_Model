'''
Author: shaoj22 935619774@qq.com
Date: 2024-01-18 17:41:18
LastEditors: shaoj22 935619774@qq.com
LastEditTime: 2024-01-24 10:03:15
FilePath: \Integrated_Picking_and_Sorting_Model\numerical_experiment\large_experiment.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''


"""




"""

import sys
sys.path.append('..')
import xlwt
import time
from generate_instances.generate_instances import generate_large_instances
from generate_instances.Integrated_Instance import Instance
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm as alg
import utils
from metaheuristic_algorithm.Integrated_ALNS import ALNS
from heuristic_algorithm.WBA_heuristic_algorithm import WBA_heuristic_algorithm
book = xlwt.Workbook(encoding='utf-8')
sheet = book.add_sheet("gurobi_solution")
# 构建excel的表头
sheet.write(0, 0, "ID")
sheet.write(0, 1, "Width")
sheet.write(0, 2, "Length")
sheet.write(0, 3, "SKUs")
sheet.write(0, 4, "Robots")
sheet.write(0, 5, "Stations")
sheet.write(0, 6, "Orders")


sheet.write(0, 7, "Heuristic Obj")
sheet.write(0, 8, "CPU time")

sheet.write(0, 9, "ALNS Obj")
sheet.write(0, 10, "Gap")
sheet.write(0, 11, "CPU time")


large_instance = generate_large_instances()
num = len(large_instance)
for i in range(0, 50):
    experiment_instance = Instance(large_instance[i][0],large_instance[i][1],large_instance[i][2],large_instance[i][3],large_instance[i][4],large_instance[i][5])
    sheet.write(i + 1, 0, i + 1)
    sheet.write(i + 1, 1, large_instance[i][0])
    sheet.write(i + 1, 2, large_instance[i][1])
    sheet.write(i + 1, 3, large_instance[i][2])
    sheet.write(i + 1, 4, large_instance[i][3])
    sheet.write(i + 1, 5, large_instance[i][4])
    sheet.write(i + 1, 6, large_instance[i][5])
    # heuristic
    NNH = alg(experiment_instance)
    WBA = WBA_heuristic_algorithm(experiment_instance)
    start_time1 = time.time()
    solution_x = NNH.NNH_main()
    # solution_z = WBA.WBA_main()
    end_time1 = time.time()
    obj1 = utils.efficient_picking_evaluate(experiment_instance, solution_x)
    # obj1 = utils.picking_integrated_evaluate(experiment_instance, NNH.transfer(solution_x))
    # ALNS
    ALNS_alg = ALNS(experiment_instance, iter_num=5000)
    start_time2 = time.time()
    solution, obj2, obj_of_500 = ALNS_alg.run()
    end_time2 = time.time()
    sheet.write(i + 1, 7, obj_of_500)
    sheet.write(i + 1, 8, end_time1-start_time1)
    sheet.write(i + 1, 9, obj2)
    sheet.write(i + 1, 10, (obj_of_500-obj2)/obj_of_500)
    sheet.write(i + 1, 11, end_time2-start_time2)

    
    
    

    
    # 保存excel文件
    save_path = "D:\\Desktop\\python_code\\Integrated_Picking_and_Sorting_Model\\numerical_experiment\\large_experiment_solution.xls"
    book.save(save_path)