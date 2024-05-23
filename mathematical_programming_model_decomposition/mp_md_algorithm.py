import sys
sys.path.append('..')
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.gurobi_model_for_logic_based_master_problem import MasterProblem
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.relaxed_gurobi_model_for_x_subproblem import xRelaxedGurobiModel
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.relaxed_gurobi_model_for_y_subproblem import yRelaxedGurobiModel
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.relaxed_gurobi_model_for_z_subproblem import zRelaxedGurobiModel
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.variable import Variable
from Integrated_Picking_and_Sorting_Model.generate_instances.Integrated_Instance import Instance
from Integrated_Picking_and_Sorting_Model.heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.gurobi_model_upper_bound import GurobiModelUpperBound
from Integrated_Picking_and_Sorting_Model.mathematical_programming_model_decomposition.gurobi_model_lower_bound import GurobiModelLowerBound
import Integrated_Picking_and_Sorting_Model.utils as utils
import numpy as np


class MpMdAlgorithm:
    def __init__(self, instance):
        """  """
        self.instance = instance
        self.objective_dict = {
            'upper_bound' : 0,
            'lower_bound' : 0,
        }
        self.variable = self.get_upper_bound()
        self.get_lower_bound()
    
    def main(self):
        """  """
        variable = self.variable
        max_iter = 5
        cur_iter = 0
        while cur_iter<max_iter:
            # 求解master problem
            cur_st = self.master_problem_runner(variable)
            # 根据master problem的解，求解sub problem
            for i in range(len(cur_st)):
                if cur_st[i] == 1:
                    cur_sub_problem_idx = i
                    break
            print("第{}次运行master problem，当前需要求解的sub problem为：{}".format(cur_iter+1, cur_sub_problem_idx))
            # 求解对应的sub problem
            variable = self.sub_problem_runner(cur_sub_problem_idx, variable)
            if variable.T_list[cur_sub_problem_idx] is None:
                break
            print("第{}次运行sub problem，当前获得的解为：{}".format(cur_iter+1, variable.T_list[cur_sub_problem_idx]))
            print("更新后的input_st为：{}，更新后的T_list为：{}".format(variable.input_st, variable.T_list))
            print("--"*50)
            cur_iter += 1
            # 如果子问题解都相同且各子问题都是最优解，则原问题达到最优解
            if variable.T_list[0] == variable.T_list[1] == variable.T_list[2]:
                break
    
    def master_problem_runner(self, variable):
        """  """
        solver = MasterProblem(variable)
        st = solver.run_gurobi_model()
        
        return st

    def sub_problem_runner(self, idx, variable):
        """  """
        if idx == 0:
            solver = xRelaxedGurobiModel(self.instance, variable, time_limit=3600)
            # get solution
            try:
                obj_val, obj_bound, solution_x, solution_passX = solver.run_gurobi_model()
            except:
                obj_val, obj_bound, solution_x, solution_passX = None, None, [[]], [[]]
            if obj_val is not None:
                if obj_val <= self.objective_dict['upper_bound']:
                    self.objective_dict['upper_bound'] = obj_val
                if obj_bound >= self.objective_dict['lower_bound']:
                    self.objective_dict['lower_bound'] = obj_bound
                # update variable
                variable_dict = {
                    'passX' : solution_passX,
                    'x' : solution_x,
                    'y' : None,
                    'z' : None,
                }
                variable.set_x_variable(variable_dict)
                solution_x = np.array(solution_x)
                solution_passX = np.array(solution_passX)
                # update variable
                variable_dict = {
                    'passX' : solution_passX,
                    'x' : solution_x,
                    'y' : None,
                    'z' : None,
                }
                variable.set_x_variable(variable_dict)
                variable.set_passX_variable(variable_dict)
                variable.T_list[0] = obj_val
                variable.input_st = [0 for _ in range(variable.spn)]
                variable.input_st[0] = 1
            else:
                variable.T_list[0] = obj_val
        elif idx == 1:
            solver = yRelaxedGurobiModel(self.instance, variable)
            # get solution
            obj_val, obj_bound, solution_y = solver.run_gurobi_model()
            if obj_val <= self.objective_dict['upper_bound']:
                self.objective_dict['upper_bound'] = obj_val
            solution_y = np.array(solution_y)
            # update variable
            variable_dict = {
                'passX' : None,
                'x' : None,
                'y' : solution_y,
                'z' : None,
            }
            variable.set_y_variable(variable_dict)
            variable.T_list[1] = obj_val
            variable.input_st = [0 for _ in range(variable.spn)]
            variable.input_st[1] = 1
        else:
            solver = zRelaxedGurobiModel(self.instance, variable)
            # get solution
            obj_val, obj_bound, solution_z = solver.run_gurobi_model()
            if obj_val <= self.objective_dict['upper_bound']:
                self.objective_dict['upper_bound'] = obj_val
            solution_z = np.array(solution_z)
            # update variable
            variable_dict = {
                'passX' : None,
                'x' : None,
                'y' : None,
                'z' : solution_z,
            }
            variable.set_z_variable(variable_dict)
            variable.T_list[2] = obj_val
            variable.input_st = [0 for _ in range(variable.spn)]
            variable.input_st[2] = 1
        
        return variable

    def get_upper_bound(self):
        """  """
        upper_bound_tool = GurobiModelUpperBound(self.instance, iter_num=30000)
        solution, obj = upper_bound_tool.runner()
        self.objective_dict['upper_bound'] = obj
        picking_solution = solution['picking']
        sorting_solution = solution['sorting']
        # 构造初始variable
        variable = Variable(self.instance)
        x_val, y_val, z_val, passX_val = utils.solution_transfer(self.instance, picking_solution, sorting_solution)
        variable_dict = {
                'passX' : passX_val,
                'x' : x_val,
                'y' : y_val,
                'z' : z_val
            }
        variable.set_passX_variable(variable_dict)
        variable.set_x_variable(variable_dict)
        variable.set_y_variable(variable_dict)
        variable.set_z_variable(variable_dict)

        return variable

    def get_lower_bound(self):
        """  """
        lower_bound_tool = GurobiModelLowerBound(self.instance)
        obj = lower_bound_tool.runner()
        self.objective_dict['lower_bound'] = obj

    def runner(self):
        """  """
        self.main()

        return self.objective_dict


if __name__ == "__main__":
    w_num = 6
    l_num = 8
    bins_num = 16
    robot_num = 15
    picking_station_num = 4
    orders_num = 10
    problem = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    mp_md_algorithm = MpMdAlgorithm(problem)
    objective_dict = mp_md_algorithm.runner()
    print(objective_dict['upper_bound'])
    print(objective_dict['lower_bound'])
