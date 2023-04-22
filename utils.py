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

