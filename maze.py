import path_planning as plan
import numpy as np
import math
from serial import Serial
import logging
import util
from util import grid_to_gantry, gantry_to_grid
import threading
import sys
import time
import gantry
import matplotlib.pyplot as plt
import cv2
import json

grid_size_x = 400
grid_size_y = 200
logger = util.get_logger()

def grid_to_world(
    row,
    col,
    grid_size_x=grid_size_x,
    grid_size_y=grid_size_y,
    world_size_x=0.46,
    world_size_y=0.26,
):
    # cell_size_x = world_size_x / grid_size_x
    # cell_size_y = world_size_y / grid_size_y
    # x = ((col + 0.5) * cell_size_x) * 1000
    # y = ((row + 0.5) * cell_size_y) * 1000
    x = col + 30 
    y = row - 15
    return (x, y)


camera_thread = threading.Thread(target=util.locate_bots)
# camera_thread.daemon = True
camera_thread.start()

my_gantry = gantry.Gantry()

while util.get_finished_walls() is None:
    # print(util.get_finished_walls())
    pass
grid = util.get_finished_walls()

grid[0, :] = 1
grid[-1, :] = 1
grid[:, 0] = 1
grid[:, -1] = 1

grid = np.abs(grid)

grid = grid[::-1]

grid = [row[::-1] for row in grid]

np.set_printoptions(threshold=sys.maxsize)

#print(grid)

astar = plan.AStar(grid)

start_grid = (80, 27)
goal_grid = (117, 240)
gant_start = grid_to_gantry(start_grid[1], start_grid[0])
my_gantry.go_to_position(gant_start[0], gant_start[1])


path = astar.find_path(start_grid, goal_grid)

for waypoint in path:
    print(waypoint)
    point = grid_to_gantry(waypoint[1], waypoint[0])

    my_gantry.go_to_position(point[0], point[1])
    util.set_gantry(point[0], point[1])
    
try:
    while True:
        pass
except KeyboardInterrupt:
    util.TRAVERSING_MAZE = False
    my_gantry.go_to_position(0, 0)
    