import time
from serial import Serial
import logging
import sys
import util
from gantry import Gantry

USING_GAMEPAD = True
util.get_logger().info("Setting up connections")

if USING_GAMEPAD:
    from gamepad import Gamepad
    gamepad = Gamepad()

gantry = Gantry()


myinput=1

while 1 :
    # get keyboard input
    if USING_GAMEPAD:
        gamepad.read_gamepad()
        xSpeed = gamepad.get_analogL_x()
        ySpeed = gamepad.get_analogL_y()
        if xSpeed == 0 and ySpeed == 0:
            continue
        xPos = gantry.get_curr_pos()[1]
        yPos = gantry.get_curr_pos()[0]
        xPos += xSpeed * 4
        yPos += ySpeed * 1

        gcode = f"G00 X{xPos} Z{yPos}"
        gantry.go_to_position(xPos, yPos)
    else:
        gcode = input(">> ")

        gantry.send_gcode(gcode)
        gantry.waitForResponse()
    # sendGCode(gcode)
    # waitForResponse("Count")


# G00 X20.00 Y0.00 Z200.00 This means to go to these coordinates as fast as possible
# G01 X20.00 Y0.00 Z200.00 F100 This means to go to these coordinates at the speed designated
# M203 X500.00 Y500.00 Z5.00 E25.00 set maximum feedrates for each thing. 
# M114 might be get the current position of the gantry. 