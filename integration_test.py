import path_planning as plan
import numpy as np
from serial import Serial
import logging

grid_size_x = 400
gird_size_y = 200

def grid_to_world(row, col, grid_size_x = grid_size_x, grid_size_y = grid_size_y, world_size_x=0.46, world_size_y = .26):
    cell_size_x = world_size_x / grid_size_x
    cell_size_y = world_size_y / grid_size_y
    x = (col + 0.5) * cell_size_x
    y = (row + 0.5) * cell_size_y
    return (x, y)


grid = np.zeros(grid_size_x, gird_size_y)


grid[0, :] = 1
grid[-1, :] = 1
grid[:, 0] = 1
grid[:, -1] = 1

ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

astar = plan.AStar(grid)

start_grid = [20, 30]
goal_grid = [90, 120]

path = astar.find_path(start_grid, goal_grid)

def sendGCode(gcode):
    gcode = gcode + "\n"
    # logger.debug(f"Sending Command: {gcode}")
    ser.write(gcode.encode())
    
for waypoint in path:
    point = grid_to_world(waypoint[0], waypoint[1])
    
    gcode = f"G00 X{point[1]} Z{point[0]}"
    