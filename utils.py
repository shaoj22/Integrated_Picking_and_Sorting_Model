'''
File: utils.py
Project: HIP_Robot
Description: 
-----
Author: CharlesLee
Created Date: Thursday April 20th 2023
'''

import numpy as np
import matplotlib.pyplot as plt
import math


class DrawTools:
    def __init__(self, pixel_size=1):
        self.psize = pixel_size

    def draw_pixel(self, ax, row, col, facecolor, edgecolor='black'):
        rect = plt.Rectangle(
            (row * self.psize, col * self.psize),
            self.psize,
            self.psize,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=1
        )
        ax.add_patch(rect)

    def draw_robot(self, ax, row, col, fill, facecolor='y', edgecolor='black'):
        circle = plt.Circle(
            ((row + 0.5) * self.psize, (col + 0.5) * self.psize),
            self.psize / 2.5,
            fill=fill,
            facecolor=facecolor,
            edgecolor=edgecolor,
            alpha=1
        )
        ax.add_patch(circle)

    def draw_map(self, ax, map):
        for idx in range(map.idx_num):
            x, y = map.idx2xy[idx]
            type = map.idx2type[idx]
            # mirror y axis in picture
            # y = map.map_width - y - 1
            # draw each type
            if type == "pod":
                self.draw_pixel(ax, x, y, facecolor='y') # 货架
            elif type == "aisle":
                self.draw_pixel(ax, x, y, facecolor='w') # 通道
            elif type == "pickerIn":
                self.draw_pixel(ax, x, y, facecolor='g') # 入口
            elif type == "pickerOut":
                self.draw_pixel(ax, x, y, facecolor='r') # 出口
        plt.xlim(0, map.map_length * self.psize)
        plt.ylim(0, map.map_width * self.psize)
        plt.axis("off")
    
    def draw_instance(self, ax, instance):
        self.draw_map(ax, instance.map)
        for robot in instance.robots:
            row, col = instance.map.idx2xy[robot["pos_idx"]]
            self.draw_robot(ax, row, col, "r")
    
    def draw_edge(self, ax, map, edge, color='r'):
        x1, y1 = map.idx2xy[edge[0]]
        x2, y2 = map.idx2xy[edge[1]]
        plt.arrow(x1+self.psize/2, y1+self.psize/2, x2-x1, y2-y1, head_width=0.2, head_length=0.2, fc=color, ec=color)
    
    def draw_routes(self, ax, map, routes):
        cmap = plt.get_cmap('Set1')
        colors = [cmap(i) for i in np.linspace(0, 1, len(routes))]
        for ri in range(len(routes)):
            for edge in routes[ri]:
                self.draw_edge(ax, map, edge, color=colors[ri])

def model2instance_routes(model, picking_instance):
    # get picking_instance idx routes
    routes = [[[i, j] for i in picking_instance.N for j in picking_instance.N if model.getVarByName(f"x[{i},{j}]").X >= 0.5 and model.getVarByName(f"passX[{i},{k}]").X >= 0.5] for k in range(picking_instance.robotNum)]
    return routes

def instance_routes2map_routes(picking_instance, routes):
    # preprocess picking_instance routes to map routes for render
    for route in routes:
        i = 0
        while i < len(route):
            edge = route[i]
            # delete edge from node to robot start
            if edge[1] in picking_instance.W: 
                route.pop(i)
                continue
            # transfer to map idx routes
            for j in range(2):
                edge[j] = picking_instance.nodes[edge[j]]["pos_idx"]
            i += 1
    return routes

def integrated_evaluate(integrated_instance, x_val, y_val, z_val):
    """ evaluate solution with gurobi model

    Args:
        integrated_instance (Integrated_Gurobi_Model): integrated instance to build model
        x_val (list/ndarray[i,j,k]): values of variables x
        y_val (list/ndarray[i,p]): values of variables y
        z_val (list/ndarray[o,p]): values of variables z

    Returns:
        obj: objective value of solution
    """
    from Integrated_Gurobi_Model import Integrated_Gurobi_Model
    import gurobipy as gp
    x_val = np.array(x_val)
    y_val = np.array(y_val)
    z_val = np.array(z_val)
    # 1. build model
    model_builder = Integrated_Gurobi_Model(integrated_instance)
    model = gp.Model("Evaluate_Integrated_Model")
    # 2. set variables value
    info = model_builder.build_model(model)
    x = info["x"]
    y = info["y"]
    z = info["z"]
    for i in integrated_instance.N:
        for j in integrated_instance.N:
            for k in integrated_instance.K:
                if x_val[i, j, k] == 1:
                    x[i, j, k].setAttr("LB", 1)
                elif x_val[i, j, k] == 0:
                    x[i, j, k].setAttr("UB", 0)
    for i in range(integrated_instance.n):
        for p in range(integrated_instance.P):
            if y_val[i, p] == 1:
                y[i, p].setAttr("LB", 1)
            elif y_val[i, p] == 0:
                y[i, p].setAttr("UB", 0)
    for o in range(integrated_instance.O):
        for p in range(integrated_instance.P):
            if z_val[o, p] == 1:
                z[o, p].setAttr("LB", 1)
            elif z_val[o, p] == 0:
                z[o, p].setAttr("UB", 0)
    # 3. solve model
    model.setParam("OutputFlag", 0)
    model.optimize()
    return model.ObjVal

def build_picking_evaluate_model(picking_instance, x_val):
    """ build gurobi model for evaluate picking solution

    Args:
        picking_instance (Picking_Gurobi_Model): picking instance to build model
        x_val (list/ndarray[i,j]): values of variables x

    Returns:
        model: gurobi model
    """
    from Picking_Gurobi_Model import Picking_Gurobi_Model
    import gurobipy as gp
    # 1. build model
    model_builder = Picking_Gurobi_Model(picking_instance)
    model = gp.Model("Evaluate_Picking_Model")
    # 2. set variables value
    info = model_builder.build_model(model)
    x = info["x"]
    for i in picking_instance.N:
        for j in picking_instance.N:
            for k in picking_instance.K:
                if x_val[i, j, k] == 1:
                    x[i, j, k].setAttr("LB", 1)
                elif x_val[i, j, k] == 0:
                    x[i, j, k].setAttr("UB", 0)
    # 3. update model
    model.update()
    return model

def picking_evaluate(picking_instance, x_val):
    """ evaluate solution with gurobi model

    Args:
        picking_instance (Picking_Gurobi_Model): picking instance to build model
        x_val (list/ndarray[i,j]): values of variables x

    Returns:
        obj: objective value of solution
    """
    model = build_picking_evaluate_model(picking_instance, x_val) 
    model.setParam("OutputFlag", 0)
    model.optimize()
    if model.Status == 3:
        return 1e6
    else:
        return model.ObjVal

def build_picking_integrated_evluate_model(integrated_instance, x_val):
    """ build gurobi model for evaluate picking solution

    Args:
        integrated_instance (Integrated_Gurobi_Model): integrated instance to build model
        x_val (list/ndarray[i,j]): values of variables x

    Returns:
        model: gurobi model
    """
    from Integrated_Gurobi_Model import Integrated_Gurobi_Model
    import gurobipy as gp
    # 1. build model
    model_builder = Integrated_Gurobi_Model(integrated_instance)
    model = gp.Model("Evaluate_Picking_Integrated_Model")
    # 2. set variables value
    info = model_builder.build_model(model)
    x = info["x"]
    for i in integrated_instance.N:
        for j in integrated_instance.N:
            for k in integrated_instance.K:
                if x_val[i, j, k] == 1:
                    x[i, j, k].setAttr("LB", 1)
                elif x_val[i, j, k] == 0:
                    x[i, j, k].setAttr("UB", 0)
    # 3. update model
    model.update()
    return model

def picking_integrated_evaluate(integrated_instance, x_val):
    """ evaluate solution with gurobi model

    Args:
        instance (Integrated_Gurobi_Model): integrated instance to build model
        x_val (list/ndarray[i,j]): values of variables x

    Returns:
        obj: objective value of solution
    """
    model = build_picking_integrated_evluate_model(integrated_instance, x_val) 
    model.setParam("OutputFlag", 0)
    model.optimize()
    return model.ObjVal

def efficient_picking_evaluate(picking_instance, solution):
    obj = 0
    # calculate pass time of each node and check time_windows
    timeMatrix = picking_instance.timeMatrix
    passTime = np.zeros(picking_instance.nodeNum)
    node2ki = {}
    P2_list = []
    for ri, route in enumerate(solution):
        cur_time = 0
        for i in range(1, len(route)):
            cur_time += picking_instance.nodes[route[i-1]]["serviceTime"] + timeMatrix[route[i-1], route[i]]
            cur_time = max(cur_time, picking_instance.nodes[route[i]]["readyTime"])
            if cur_time > picking_instance.nodes[route[i]]["dueTime"]:
                obj += 10000
            passTime[route[i]] = cur_time
            node2ki[route[i]] = (ri, i)
            if picking_instance.node2type[route[i]] == "P2":
                P2_list.append(route[i])
    # check feasibility and fix passTime of P2, D2
    # 1. D12 later than P12
    for ni in range(2*picking_instance.n): # for P12 
        if passTime[ni] > passTime[ni+2*picking_instance.n]:
            obj += 10000
    # 2. P2 later than D1, and fix pick_time
    for ni in P2_list:
        if passTime[ni] < passTime[ni+picking_instance.n] + picking_instance.pick_time:
            k, start_i = node2ki[ni]
            route = solution[k]
            extra_time = passTime[ni+picking_instance.n] + picking_instance.pick_time - passTime[ni]
            for i in range(start_i, len(route)):
                passTime[route[i]] += extra_time
    # 3. check capacity
    for k in range(picking_instance.robotNum):
        load = 0
        for i in range(1, len(solution[k])):
            load += picking_instance.nodes[solution[k][i]]["demand"]
            if load > picking_instance.capacity:
                obj += 10000
    obj += np.max(passTime)

    return obj



