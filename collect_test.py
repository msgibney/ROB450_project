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

# my_gantry.mag_pattern('B', 0, 1)

patterns = ['B', 'U', 'X', 'T', 'O', 'l', 'vl', 'hl', 'L', 'mL', 'J', 'mJ']

pattern = 0
ori = 0
amplitude = 1

try:
    while util.TRAVERSING_MAZE:
        try:
            frame = util.frame_queue.get(timeout=0.05)
            cv2.imshow("live_feed", cv2.resize(frame, (1536, 864)))
        except Empty:
            frame = None

        if frame is not None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        util.collect_bots(my_gantry)
        TRAVERSING_MAZE = False

finally:
    # util.frame_queue.put(None)
    camera_thread.join()
    my_gantry.go_to_position(0, 0)
    cv2.destroyAllWindows()
    