'''
File: gurobi_model_upper_bound.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
relaxed all variable except x.
----------
Author: 626
Created Date: 2024.5.21
'''


from Integrated_Picking_and_Sorting_Model.metaheuristic_algorithm.Integrated_ALNS import ALNS
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


class GurobiModelUpperBound:
    def __init__(self, instance, iter_num=30000):
        """  """
        self.instance = instance
        self.iter_num = iter_num
    
    def main(self):
        """  """
        alg = ALNS(self.instance, iter_num=self.iter_num)
        solution, obj, obj_of_500 = alg.run()

        return solution, obj

    def runner(self):
        """  """
        solution, obj = self.main()

        return solution, obj

if __name__ == '__main__':
    # create instance
    w_num = 6
    l_num = 6
    bins_num = 10
    robot_num = 5
    picking_station_num = 5
    orders_num = 5
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    upper_bound_tool = GurobiModelUpperBound(instance)
    solution = upper_bound_tool.runner()
    print(solution)