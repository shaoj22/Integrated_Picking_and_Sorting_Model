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


from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.linear_relaxed_gurobi_model import LinearRelaxedGurobiModel
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance


class GurobiModelLowerBound:
    def __init__(self, instance):
        """  """
        self.instance = instance
    
    def main(self):
        """  """
        alg = LinearRelaxedGurobiModel(self.instance, init_flag=True)
        try:
            obj = alg.run_gurobi_model()
        except:
            obj = 0

        return obj

    def runner(self):
        """  """
        obj = self.main()

        return obj

if __name__ == '__main__':
    # create instance
    w_num = 6
    l_num = 6
    bins_num = 10
    robot_num = 5
    picking_station_num = 5
    orders_num = 5
    instance = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    lower_bound_tool = GurobiModelLowerBound(instance)
    solution = lower_bound_tool.runner()
    print(solution)