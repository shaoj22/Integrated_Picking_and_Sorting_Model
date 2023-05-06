'''
File: gurobi_experiment.py
Created Date: April 25th 2023
'''
import Integrated_Gurobi_Model
import Integrated_Instance
import xlwt
import generate_instances

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
sheet.write(0, 8, "objval")
sheet.write(0, 9, "objBound")
sheet.write(0, 10, "Gap")
sheet.write(0, 11, "Time")
# get small instances
instance = generate_instances.generate_small_instances()
# 求解并输出每个算例
for i in range(35,len(instance)):
    gurobi_experiment_instance = Integrated_Instance.Instance(instance[i][0],instance[i][1],instance[i][2],instance[i][3],instance[i][4],instance[i][5])
    gurobi_alg = Integrated_Gurobi_Model.Integrated_Gurobi_Model(gurobi_experiment_instance, time_limit = 360)
    model, obj, Time, objval, objBound, Gap = gurobi_alg.run_gurobi()
    # 输出到excel中
    sheet.write(i + 1, 0, i + 1)
    sheet.write(i + 1, 1, instance[i][0])
    sheet.write(i + 1, 2, instance[i][1])
    sheet.write(i + 1, 3, instance[i][2])
    sheet.write(i + 1, 4, instance[i][3])
    sheet.write(i + 1, 5, instance[i][4])
    sheet.write(i + 1, 6, instance[i][5])
    sheet.write(i + 1, 7, obj)
    sheet.write(i + 1, 8, objval)
    sheet.write(i + 1, 9, objBound)
    sheet.write(i + 1, 10, Gap)
    sheet.write(i + 1, 11, Time)
    # 保存excel文件
    save_path = "D:\PythonProjet\SF-learning\HaiSystem_Integrated_Model\Integrated_Picking & Sorting_Model\gurobi_solution2.xls"
    book.save(save_path)
