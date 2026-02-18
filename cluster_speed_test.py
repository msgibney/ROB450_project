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

start_grid = (50, 100)
gant_start = util.grid_to_gantry(start_grid[0], start_grid[1])
my_gantry.go_to_position(gant_start[0], gant_start[1])
util.set_gantry(gant_start[0], gant_start[1])

util.prompt_for_filenames()

camera_thread = threading.Thread(target=util.locate_bots)
camera_thread.daemon = False
# camera_thread.daemon = True
camera_thread.start()
done = False

#print(grid)

go_grid = (250, 100)
gant_go = util.grid_to_gantry(go_grid[0], go_grid[1])
my_gantry.go_to_position_at_speed(gant_go[0], gant_go[1], 25)
util.set_gantry(gant_go[0], gant_go[1])

go_grid = (50, 100)
gant_go = util.grid_to_gantry(go_grid[0], go_grid[1])
my_gantry.go_to_position_at_speed(gant_go[0], gant_go[1], 25)
util.set_gantry(gant_go[0], gant_go[1])

my_gantry.go_to_position_at_speed(0, 0, 500)
util.TRAVERSING_MAZE = False
camera_thread.join()
cv2.destroyAllWindows()