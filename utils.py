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
        color_list = ["r", "g", "b", "c", "m", "k", "w", "nav"]
        for ri in range(len(routes)):
            for edge in routes[ri]:
                self.draw_edge(ax, map, edge, color=color_list[ri])

def model2routes(model, instance):
    # get instance idx routes
    routes = [[[i, j] for i in instance.N for j in instance.N if model.getVarByName(f"x[{i},{j},{k}]").X == 1] for k in range(instance.robotNum)]
    # preprocess
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
