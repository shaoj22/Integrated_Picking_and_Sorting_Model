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

def model2instance_routes(model, instance):
    # get instance idx routes
    routes = [[[i, j] for i in instance.N for j in instance.N if model.getVarByName(f"x[{i},{j},{k}]").X == 1] for k in range(instance.robotNum)]
    return routes

def instance_routes2map_routes(instance, routes):
    # preprocess instance routes to map routes for render
    for route in routes:
        i = 0
        while i < len(route):
            edge = route[i]
            # delete edge from node to robot start
            if edge[1] in instance.W: 
                route.pop(i)
                continue
            # transfer to map idx routes
            for j in range(2):
                edge[j] = instance.nodes[edge[j]]["pos_idx"]
            i += 1
    return routes

def evaluate(instance, x_val, y_val, z_val):
    """ evaluate solution with gurobi model

    Args:
        instance (Integrated_Gurobi_Model): instance to build model
        x_val (list/ndarray[i,j,k]): values of variables x
        y_val (list/ndarray[i,p]): values of variables y
        z_val (list/ndarray[o,p]): values of variables z

    Returns:
        obj: objective value of solution
    """
    # 1. build model
    model_builder = Integrated_Gurobi_Model(instance)
    model = gp.Model("Evaluate_Integrated_Model")
    # 2. set variables value
    info = model_builder.build_model(model)
    x = info["x"]
    y = info["y"]
    z = info["z"]
    for i in instance.N:
        for j in instance.N:
            for k in instance.K:
                x[i, j, k].setAttr("X", x_val[i, j, k])
    for i in range(instance.n):
        for p in instance.P:
            y[i, p].setAttr("X", y_val[i, p])
    for o in instance.O:
        for p in instance.P:
            z[o, p].setAttr("X", z_val[o, p])
    # 3. solve model
    model.setParam("OutputFlag", 0)
    model.optimize()
    return model.ObjVal