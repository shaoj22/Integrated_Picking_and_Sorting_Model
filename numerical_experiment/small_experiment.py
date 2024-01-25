"""




"""

import sys
sys.path.append('..')
import xlwt
import time
from generate_instances.generate_instances import generate_small_instances
from generate_instances.Integrated_Instance import Instance
from gurobi_model.Integrated_Gurobi_Model import Integrated_Gurobi_Model
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm as alg
import utils
from metaheuristic_algorithm.Integrated_ALNS import ALNS

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

sheet.write(0, 7, "Upper Bound")
sheet.write(0, 8, "Lower Bound")
sheet.write(0, 9, "MIP Gap")
sheet.write(0, 10, "CPU time")

sheet.write(0, 11, "Heuristic Obj")
sheet.write(0, 12, "Gap UB")
sheet.write(0, 13, "Gap LB")
sheet.write(0, 14, "CPU time")

sheet.write(0, 15, "ALNS Obj")
sheet.write(0, 16, "Gap UB")
sheet.write(0, 17, "Gap LB")
sheet.write(0, 18, "CPU time")


small_instance = generate_small_instances()
num = len(small_instance)
for i in range(0, num):
    experiment_instance = Instance(small_instance[i][0],small_instance[i][1],small_instance[i][2],small_instance[i][3],small_instance[i][4],small_instance[i][5])
    
    # gurobi 
    gurobi_alg = Integrated_Gurobi_Model(experiment_instance, time_limit = 360)
    model, obj, Time, objVal, objBound, Gap = gurobi_alg.run_gurobi()
    sheet.write(i + 1, 0, i + 1)
    sheet.write(i + 1, 1, small_instance[i][0])
    sheet.write(i + 1, 2, small_instance[i][1])
    sheet.write(i + 1, 3, small_instance[i][2])
    sheet.write(i + 1, 4, small_instance[i][3])
    sheet.write(i + 1, 5, small_instance[i][4])
    sheet.write(i + 1, 6, small_instance[i][5])
    sheet.write(i + 1, 7, objVal)
    sheet.write(i + 1, 8, objBound)
    sheet.write(i + 1, 9, Gap)
    sheet.write(i + 1, 10, Time)
    # heuristic
    NNH = alg(experiment_instance)
    start_time1 = time.time()
    solution = NNH.NNH_main()
    end_time1 = time.time()
    obj1 = utils.picking_integrated_evaluate(experiment_instance, NNH.transfer(solution))
    sheet.write(i + 1, 11, obj1)
    sheet.write(i + 1, 12, (obj1-objVal)/obj1)
    sheet.write(i + 1, 13, (obj1-objBound)/obj1)
    sheet.write(i + 1, 14, end_time1-start_time1)
    # ALNS
    ALNS_alg = ALNS(experiment_instance, iter_num=3000)
    start_time2 = time.time()
    solution, obj2 = ALNS_alg.run()
    end_time2 = time.time()
    sheet.write(i + 1, 15, obj2)
    sheet.write(i + 1, 16, (obj2-objVal)/obj2)
    sheet.write(i + 1, 17, (obj2-objBound)/obj2)
    sheet.write(i + 1, 18, end_time2-start_time2)

    
    
    

    
    # 保存excel文件
    save_path = "D:\\Desktop\\python_code\\Integrated_Picking_and_Sorting_Model\\numerical_experiment\\small_experiment_solution.xls"
    book.save(save_path)