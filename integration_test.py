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
    x = ((col + 0.5) * cell_size_x)*1000
    y = ((row + 0.5) * cell_size_y)*1000
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

while (util.get_finished_walls() is None):
    # print(util.get_finished_walls())
    pass
grid = util.get_finished_walls()
print(grid)


grid[0, :] = 1
grid[-1, :] = 1
grid[:, 0] = 1
grid[:, -1] = 1

ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

astar = plan.AStar(grid)

while util.get_coordinates() == []:
    pass

bots = util.get_coordinates()
print(bots)

goal_grid = bots[2]

print(goal_grid)

start_grid = [20, 30]

path = astar.find_path(start_grid, goal_grid)

def sendGCode(gcode):
    gcode = gcode + "\n"
    # logger.debug(f"Sending Command: {gcode}")
    ser.write(gcode.encode())
    
def clearSerial():
    while ser.inWaiting() > 0:
        out = ser.readline()

def waitForResponse(response):
    getLoc = "M114\n"
    ser.write(getLoc.encode())
    # logger.debug(f"Waiting for: {response}")

    gotResponse = False

    while not gotResponse:
        # out = b''
        while ser.inWaiting() > 0:
            out = ser.readline()
            if out != '':
                # logger.debug(out.decode())
                # break
                if response in out.decode():
                    gotResponse = True
                    break
        # time.sleep(0.001)
while not ser.isOpen():
    time.sleep(1)

time.sleep(3) # Give the printer a chance to get ready to receive messages

clearSerial()

# logger.info('Enter your commands below.')
XMax = 1000
ZMax = 500
setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
ser.write(setMaximumSpeeds.encode())

goHome = "G28\n"
ser.write(goHome.encode())
# logger.info("Please wait till home is set")

waitForResponse("X:0.00 Y:0.00 Z:0.00")
# logger.info("Home is set")

    
for waypoint in path:
    point = grid_to_world(waypoint[1], waypoint[0])
    
    print(point)
    
    gcode = f"G00 X{point[1]} Z{point[0]}"
    sendGCode(gcode)
    waitForResponse("Count")