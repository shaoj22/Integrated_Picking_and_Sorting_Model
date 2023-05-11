import time
from Sorting_Instance import Instance
import numpy as np
import generate_instances
import xlwt
import WBA_heuristic_algorithm
import Integrated_Instance



if __name__ == "__main__":
    T = 0 
    small = generate_instances.generate_medium_instances()
    # 产生每个算例
    book = xlwt.Workbook(encoding='utf-8')
    sheet = book.add_sheet("VNS_solution")
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
    for i in range(0,len(small)):
        start_time = time.time()
        instance = Integrated_Instance.Instance(small[i][0], small[i][1], small[i][2], small[i][3], small[i][4], small[i][5])
        WBA = WBA_heuristic_algorithm.WBA_heuristic_algorithm(instance)
        problem = Instance(T, small[i][2], small[i][4], small[i][5])
        z_val = WBA.WBA_main()
        Obj = WBA_heuristic_algorithm.sorting_evaluate(Instance(0, small[i][2], small[i][4], small[i][5]), z_val)
        print(Obj)
        end_time = time.time()
        Time = end_time - start_time
        sheet.write(i + 1, 0, i + 1)
        sheet.write(i + 1, 1, small[i][0])
        sheet.write(i + 1, 2, small[i][1])
        sheet.write(i + 1, 3, small[i][2])
        sheet.write(i + 1, 4, small[i][3])
        sheet.write(i + 1, 5, small[i][4])
        sheet.write(i + 1, 6, small[i][5])
        sheet.write(i + 1, 7, Obj)
        sheet.write(i + 1, 8, Time)
        # 保存excel文件
        save_path = "C:\\Users\\93561\\Desktop\\code\\Integrated_Picking & Sorting_Model\\VNS_solution_m.xls"
        book.save(save_path)