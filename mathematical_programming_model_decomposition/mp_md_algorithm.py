import sys
sys.path.append('..')
from mathematical_programming_model_decomposition.gurobi_model_for_logic_based_master_problem import masterProblem
from mathematical_programming_model_decomposition.relaxed_gurobi_model_for_x_subproblem import xRelaxedGurobiModel
from mathematical_programming_model_decomposition.relaxed_gurobi_model_for_y_subproblem import yRelaxedGurobiModel
from mathematical_programming_model_decomposition.relaxed_gurobi_model_for_z_subproblem import zRelaxedGurobiModel
from mathematical_programming_model_decomposition.variable import Variable
from generate_instances.Integrated_Instance import Instance
from heuristic_algorithm.NNH_heuristic_algorithm import NNH_heuristic_algorithm
import utils
import numpy as np


class mpMdAlgorithm:
    def __init__(self, instance, variable):
        """  """
        self.instance = instance
        self.variable = variable
    
    def main(self):
        """  """
        variable = self.variable
        max_iter = 10
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
            print("第{}次运行sub problem，当前获得的解为：{}".format(cur_iter+1, variable.T_list[cur_sub_problem_idx]))
            print("更新后的input_st为：{}，更新后的T_list为：{}".format(variable.input_st, variable.T_list))
            print("--"*50)
            cur_iter += 1
            # 如果子问题解都相同且各子问题都是最优解，则原问题达到最优解
            if variable.T_list[0] == variable.T_list[1] == variable.T_list[2]:
                break
    
    def master_problem_runner(self, variable):
        """  """
        solver = masterProblem(variable)
        st = solver.run_gurobi_model()
        
        return st

    def sub_problem_runner(self, idx, variable):
        """  """
        if idx == 0:
            solver = xRelaxedGurobiModel(self.instance, variable, time_limit=180)
            # get solution
            objVal, solution_x = solver.run_gurobi_model()
            solution_x = np.array(solution_x)
            # update variable
            variable_dict = {
            'x' : solution_x,
            'y' : None,
            'z' : None,
            }
            variable.set_x_variable(variable_dict)
            variable.T_list[0] = objVal
            variable.input_st = [0 for _ in range(variable.spn)]
            variable.input_st[0] = 1
        elif idx == 1:
            solver = yRelaxedGurobiModel(self.instance, variable)
            # get solution
            objVal, solution_y = solver.run_gurobi_model()
            solution_y = np.array(solution_y)
            # update variable
            variable_dict = {
            'x' : None,
            'y' : solution_y,
            'z' : None,
            }
            variable.set_y_variable(variable_dict)
            variable.T_list[1] = objVal
            variable.input_st = [0 for _ in range(variable.spn)]
            variable.input_st[1] = 1
        else:
            solver = zRelaxedGurobiModel(self.instance, variable)
            # get solution
            objVal, solution_z = solver.run_gurobi_model()
            solution_z = np.array(solution_z)
            # update variable
            variable_dict = {
            'x' : None,
            'y' : None,
            'z' : solution_z,
            }
            variable.set_z_variable(variable_dict)
            variable.T_list[2] = objVal
            variable.input_st = [0 for _ in range(variable.spn)]
            variable.input_st[2] = 1
        
        return variable

    def runner(self):
        """  """
        self.main()


if __name__ == "__main__":
    w_num = 6
    l_num = 6
    bins_num = 5
    robot_num = 5
    picking_station_num = 5
    orders_num = 5
    problem = Instance(w_num, l_num, bins_num, orders_num, robot_num, picking_station_num)
    # 获得初始解
    picking_alg = NNH_heuristic_algorithm(problem)
    picking_solution = picking_alg.NNH_main()
    sorting_solution = [np.random.randint(problem.P) for _ in range(problem.O)]
    # 构造初始variable
    variable = Variable(problem)
    x_val, y_val, z_val = utils.solution_transfer(problem, picking_solution, sorting_solution)
    variable_dict = {
            'x' : x_val,
            'y' : y_val,
            'z' : z_val
        }
    variable.set_x_variable(variable_dict)
    variable.set_y_variable(variable_dict)
    variable.set_z_variable(variable_dict)
    # 求解
    mp_md_algorithm = mpMdAlgorithm(problem, variable)
    mp_md_algorithm.runner()
