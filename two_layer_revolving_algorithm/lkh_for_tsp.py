'''
File: operators_for_x.py
Project: Integrated_Picking_and_Sorting_Model
Description: 
----------
as the file name, here are some operators for the x variable.
----------
Author: 626
Created Date: 2024.01.02
'''


class LKH:
    def __init__(self, problem=None):
        """ ...... """
        self.problem = problem
        self.dist_matrix = problem.disMatrix
    

    def runner(self, robot_points_list):
        """ ...... """
        # create the robot's route
        robot_route = []
        # add start node
        robot_route.append(robot_points_list[0])
        robot_points_list.pop(0)
        left_node_list = robot_points_list
        # 不断加入最近的点
        while len(left_node_list) != 0:
            min_dist = float('inf')
            min_node = None
            for node in left_node_list:
                # # 若p2的对应的d1还未完成则continue
                # if self.problem.node2type[node] == "P2" and node+self.problem.n not in robot_route:
                #     continue
                # 若d的p还未完成则continue
                if self.problem.node2type[node] in ["D1", "D2"] and node-2*self.problem.n not in robot_route:
                    continue
                dist = self.dist_matrix[robot_route[-1]][node]
                if dist < min_dist:
                    # find node
                    min_dist = dist
                    min_node = node
            # add node
            robot_route.append(min_node)
            # remove node
            left_node_list.remove(min_node)

        
        return robot_route


