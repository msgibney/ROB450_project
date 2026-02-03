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

myinput=1
xPos = 0
yPos = 0

my_gantry.mag_pattern('B', 0, 1)

patterns = ['B', 'U', 'X', 'T', 'O', 'l', 'vl', 'hl', 'L', 'mL', 'J', 'mJ']

pattern = 0
ori = 0
amplitude = 1

try:
    while util.TRAVERSING_MAZE:
        gamepad.read_gamepad()
        xSpeed = gamepad.get_analogL_x()
        ySpeed = gamepad.get_analogL_y()
        lb_pressed = gamepad.get_LB()
        rb_pressed = gamepad.get_RB()
        a_pressed = gamepad.get_A()
        b_pressed = gamepad.get_B()
        x_pressed = gamepad.get_X()
        y_pressed = gamepad.get_Y()

        try:
            frame = util.frame_queue.get(timeout=0.05)
            cv2.imshow("live_feed", cv2.resize(frame, (1536, 864)))
        except Empty:
            frame = None

        if frame is not None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if(rb_pressed):
            ori += 1
            if(ori >= 4):
                ori = 0
            print('rb_pressed', ori)
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)
        elif(lb_pressed):
            ori -= 1
            if(ori <= -1):
                ori = 3
            print('lb_pressed', ori)
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)

        if(a_pressed):
            pattern += 1
            if(pattern >= len(patterns)):
                pattern = 0
            print('a_pressed', patterns[pattern])
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)
        elif(b_pressed):
            pattern -= 1
            if(pattern <= 0):
                pattern = len(patterns) - 1
            print('b_pressed', patterns[pattern])
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)

        if(x_pressed):
            amplitude += 0.05
            if(amplitude >= 1):
                amplitude = 1
            print('x_pressed', amplitude)
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)
        elif(y_pressed):
            amplitude -= 0.05
            if(amplitude <= 0):
                amplitude = 0
            print('y_pressed', amplitude)
            my_gantry.mag_pattern(patterns[pattern], ori, amplitude)
        
        if xSpeed == 0 and ySpeed == 0:
            pass
        xPos += xSpeed * 2
        yPos += ySpeed * 3
        
        if xPos < 0:
            xPos = 0
        
        if yPos < 0:
            yPos = 0

        my_gantry.go_to_position(xPos, yPos)
        util.set_gantry(my_gantry.current_pos[0], my_gantry.current_pos[1])
        util.logger.info(f"x: {xPos}, y: {yPos}")

finally:
    # util.frame_queue.put(None)
    camera_thread.join()
    my_gantry.go_to_position(0, 0)
    cv2.destroyAllWindows()
    