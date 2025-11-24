import path_planning as plan
import numpy as np
from serial import Serial
import logging
import util
import threading
import sys
import time
import gantry

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
    x = col 
    y = row
    y += 30
    x += 25
    return (x, y)


# def world_to_gantry(point):
#     return [point[0] *, ]


camera_thread = threading.Thread(target=util.locate_bots)
# camera_thread.daemon = True
camera_thread.start()
done = False

my_gantry = gantry.Gantry()

while util.get_finished_walls() is None:
    # print(util.get_finished_walls())
    pass
grid = util.get_finished_walls()

grid[0, :] = 1
grid[-1, :] = 1
grid[:, 0] = 1
grid[:, -1] = 1

try:
    astar = plan.AStar(grid)

    while util.get_coordinates() == []:
        pass

    bots = util.get_coordinates()
    logger.info(f"Bots: {bots}")

    goal_grid = bots[2]

    start_grid = (20, 30)

    goal_grid = (int(goal_grid[1]), int(goal_grid[0]))

    logger.info(f"Goal: {goal_grid}")

    path = astar.find_path(start_grid, goal_grid)

    logger.debug(f"Path: {path}")

    for waypoint in path:
        point = grid_to_world(waypoint[1], waypoint[0])

        my_gantry.go_to_position(point[0], point[1])

    done = True
except Exception as e:
    done = True
    logger.info(f"Bad things happened: {e}")
except KeyboardInterrupt:
    logger.info(f"Cancelling Operations")
    my_gantry.zero()
    done = True
logger.info(f"Goal: {goal_grid}")
logger.info("Finished")
