import time
from serial import Serial
import logging
import sys
from gantry import Gantry

USING_GAMEPAD = True
logger = logging.getLogger(__name__)

logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

logger.info("Setting up connections")
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

def clearSerial():
    while ser.inWaiting() > 0:
        out = ser.readline()

def waitForResponse(response):
    getLoc = "M114\n"
    ser.write(getLoc.encode())
    logger.debug(f"Waiting for: {response}")

    gotResponse = False

    while not gotResponse:
        # out = b''
        while ser.inWaiting() > 0:
            out = ser.readline()
            if out != '':
                logger.debug(out.decode())
                # break
                if response in out.decode():
                    gotResponse = True
                    break
        # time.sleep(0.001)
    

def sendGCode(gcode):
    gcode = gcode + "\n"
    logger.debug(f"Sending Command: {gcode}")
    ser.write(gcode.encode())

if USING_GAMEPAD:
    from gamepad import Gamepad
    gamepad = Gamepad()

gantry = Gantry()

ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

def clearSerial():
    while ser.inWaiting() > 0:
        out = ser.readline()

def waitForResponse(response):
    getLoc = "M114\n"
    ser.write(getLoc.encode())
    logger.debug(f"Waiting for: {response}")

    gotResponse = False

    while not gotResponse:
        # out = b''
        while ser.inWaiting() > 0:
            out = ser.readline()
            if out != '':
                logger.debug(out.decode())
                # break
                if response in out.decode():
                    gotResponse = True
                    break
        # time.sleep(0.001)
    

def sendGCode(gcode):
    gcode = gcode + "\n"
    logger.debug(f"Sending Command: {gcode}")
    ser.write(gcode.encode())

while not ser.isOpen():
    time.sleep(1)

time.sleep(3) # Give the printer a chance to get ready to receive messages

clearSerial()

logger.info('Enter your commands below.')
XMax = 1000
ZMax = 500
setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
ser.write(setMaximumSpeeds.encode())

goHome = "G28\n"
ser.write(goHome.encode())
logger.info("Please wait till home is set")

waitForResponse("X:0.00 Y:0.00 Z:0.00")
logger.info("Home is set")

myinput=1
xPos = 0
yPos = 0
while 1 :
    # get keyboard input
    if USING_GAMEPAD:
        gamepad.read_gamepad()
        xSpeed = gamepad.get_analogL_x()
        ySpeed = gamepad.get_analogL_y()
        if xSpeed == 0 and ySpeed == 0:
            continue
        xPos += xSpeed * 4
        yPos += ySpeed * 1
        
        if xPos < 0:
            xPos = 0
        
        if yPos < 0:
            yPos = 0

        gcode = f"G00 X{xPos} Z{yPos}"
        # gantry.go_to_position(xPos, yPos)
    else:
        gcode = input(">> ")

        # gantry.sendGCode(gcode)
        # gantry.waitForResponse()
    sendGCode(gcode)
    waitForResponse("Count")


# G00 X20.00 Y0.00 Z200.00 This means to go to these coordinates as fast as possible
# G01 X20.00 Y0.00 Z200.00 F100 This means to go to these coordinates at the speed designated
# M203 X500.00 Y500.00 Z5.00 E25.00 set maximum feedrates for each thing. 
# M114 might be get the current position of the gantry. 