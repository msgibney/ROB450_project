import path_planning as plan
import numpy as np
from serial import Serial
import logging
import util
import threading
import sys
import time
import gantry
import cv2
import signal
from gamepad import Gamepad
from queue import Empty

def stop(signum, frame):
    util.TRAVERSING_MAZE = False

signal.signal(signal.SIGINT, stop)

gamepad = Gamepad()
my_gantry = gantry.Gantry()


camera_thread = threading.Thread(target=util.locate_bots)
camera_thread.daemon = False
# camera_thread.daemon = True
camera_thread.start()
done = False
# my_gantry.go_to_position(230, 400)

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

myinput=1
xPos = 0
yPos = 0

# my_gantry.mag_pattern('l', 0, 1)

patterns = ['B', 'U', 'X', 'T', 'O', 'l', 'vl', 'hl', 'L', 'mL', 'J', 'mJ']

pattern = 0
ori = 0
amplitude = 1

all_collected = False

start_grid = (10, 10)
gant_start = util.grid_to_gantry(start_grid[0], start_grid[1])
my_gantry.go_to_position(gant_start[0], gant_start[1])
util.set_gantry(gant_start[0], gant_start[1])

try:
    while not all_collected and util.TRAVERSING_MAZE:
        try:
            frame = util.frame_queue.get(timeout=0.05)
            cv2.imshow("live_feed", cv2.resize(frame, (1536, 864)))
        except Empty:
            frame = None

        if frame is not None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        start_grid = util.camera_to_grid(util.gantry_loc[0], util.gantry_loc[1])

        start = (int(start_grid[1]), int(start_grid[0]))

        goal_grid = util.collect_bots()

        if(goal_grid == []):
            all_collected = True
            break

        goal = (int(goal_grid[1]), int(goal_grid[0]))

        # print(start, goal)

        path = astar.find_path(start, goal)

        for waypoint in path:
            point = util.grid_to_gantry(waypoint[1], waypoint[0])

            my_gantry.go_to_position(point[0], point[1])
            util.set_gantry(point[0], point[1])

        time.sleep(0.25)
        TRAVERSING_MAZE = False
    
    my_gantry.mag_pattern('d', 0, 1)
    time.sleep(1.0)
    waypoints = util.drawSAM()
    for waypoint in waypoints:
        point = util.grid_to_gantry(waypoint[0], waypoint[1])
        my_gantry.go_to_position(point[0]-25, point[1])
        util.set_gantry(point[0]-25, point[1])

finally:
    # util.frame_queue.put(None)
    camera_thread.join()
    my_gantry.go_to_position(0, 0)
    cv2.destroyAllWindows()
    