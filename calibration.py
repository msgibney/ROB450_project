import path_planning as plan
import numpy as np
from serial import Serial
import logging
import util
import threading
import sys
import time
import gantry
from gamepad import Gamepad


gamepad = Gamepad()
my_gantry = gantry.Gantry()


camera_thread = threading.Thread(target=util.calibrate_visuals)
# camera_thread.daemon = True
camera_thread.start()
done = False
# my_gantry.go_to_position(230, 400)

myinput=1
xPos = 0
yPos = 0
while True:
    gamepad.read_gamepad()
    xSpeed = gamepad.get_analogL_x()
    ySpeed = gamepad.get_analogL_y()
    if xSpeed == 0 and ySpeed == 0:
        continue
    xPos += xSpeed * 3
    yPos += ySpeed * 4
    
    if xPos < 0:
        xPos = 0
    
    if yPos < 0:
        yPos = 0

    my_gantry.go_to_position(xPos, yPos)
    util.set_gantry(my_gantry.current_pos[0], my_gantry.current_pos[1])
    util.logger.info(f"x: {xPos}, y: {yPos}")