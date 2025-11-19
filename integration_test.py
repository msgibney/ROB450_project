import path_planning as plan
import numpy as np
from serial import Serial
import logging
import util
import threading
import sys

grid_size_x = 400
grid_size_y = 200

def grid_to_world(row, col, grid_size_x = grid_size_x, grid_size_y = grid_size_y, world_size_x=0.46, world_size_y = .26):
    cell_size_x = world_size_x / grid_size_x
    cell_size_y = world_size_y / grid_size_y
    x = (col + 0.5) * cell_size_x
    y = (row + 0.5) * cell_size_y
    return (x, y)


camera_thread = threading.Thread(target=util.locate_bots)
camera_thread.start()

def exit_cleanup():
    try:
        while True:
            pass
    except KeyboardInterrupt:
        camera_thread.join()
        exit(0)

ctrl_c_polling = threading.Thread(target=exit_cleanup)
ctrl_c_polling.start()

while (not util.get_finished_walls()):
    pass
grid = util.get_finished_walls()


grid[0, :] = 1
grid[-1, :] = 1
grid[:, 0] = 1
grid[:, -1] = 1

ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

astar = plan.AStar(grid)

bots = util.get_coordinates()

goal_grid = bots[2]

print(goal_grid)

start_grid = [20, 30]

path = astar.find_path(start_grid, goal_grid)

def sendGCode(gcode):
    gcode = gcode + "\n"
    # logger.debug(f"Sending Command: {gcode}")
    ser.write(gcode.encode())
    
for waypoint in path:
    point = grid_to_world(waypoint[0], waypoint[1])
    
    gcode = f"G00 X{point[1]} Z{point[0]}"
    