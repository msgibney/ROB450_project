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

from gamepad import Gamepad

def stop(signum, frame):
    util.CALIBRATING = False

signal.signal(signal.SIGINT, stop)


gamepad = Gamepad()
my_gantry = gantry.Gantry()


camera_thread = threading.Thread(target=util.calibrate_visuals)
camera_thread.daemon = False
# camera_thread.daemon = True
camera_thread.start()
done = False
# my_gantry.go_to_position(230, 400)

myinput=1
xPos = 0
yPos = 0

cam_points = np.array([
    [130.4296875, 84.75],
    [60.890625, 62.4375],
    [65.3203125, 236.875],
    [109.3828125, 172.46875],
    [169.734375, 235.0625],
    [223.78125, 144.65625],
    [221.671875, 56.6875],
    [393.6328125, 52.96875],
    [338.5546875, 116.625],
    [319.7109375, 204.1875],
    [403.5703125, 228.125]
])

grid_points = np.array([
    [120, 50],
    [60, 30],
    [60, 170],
    [100, 120],
    [150, 170],
    [200, 100],
    [200, 30],
    [350, 30],
    [300, 80],
    [280, 150],
    [350, 170]
])

N = cam_points.shape[0]
A = np.zeros((2*N, 6))
B = np.zeros(2*N)

for i in range(N):
    x, y = cam_points[i]
    gx, gy = grid_points[i]
    A[2*i] = [x, y, 1, 0, 0, 0]
    B[2*i] = gx
    A[2*i+1] = [0, 0, 0, x, y, 1]
    B[2*i+1] = gy

params = np.linalg.lstsq(A, B, rcond=None)[0]
a, b, tx, c, d, ty = params

print("Affine transform parameters:")
print("a =", a, "b =", b, "tx =", tx)
print("c =", c, "d =", d, "ty =", ty)

try:
    while util.CALIBRATING:
        gamepad.read_gamepad()
        xSpeed = gamepad.get_analogL_x()
        ySpeed = gamepad.get_analogL_y()

        try:
            frame = util.frame_queue.get(timeout=0.05)
            cv2.imshow("live_feed", cv2.resize(frame, (1536, 864)))
        except Empty:
            frame = None

        if frame is not None:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
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
finally:
    # util.frame_queue.put(None)
    camera_thread.join()
    my_gantry.go_to_position(0, 0)
    cv2.destroyAllWindows()