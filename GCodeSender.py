import time
from serial import Serial
from gamepad import Gamepad
import logging

logger = logging.getLogger(__name__)

logger.info("Setting up connections")
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

gamepad = Gamepad()

while not ser.isOpen():
    time.sleep(1)

time.sleep(3) # Give the printer a chance to get ready to receive messages

logger.info('Enter your commands below.\r\nInsert "exit" to leave the application.')
XMax = 1000
ZMax = 500
setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
ser.write(setMaximumSpeeds.encode())

goHome = "G28\n"
ser.write(goHome.encode())
logger.info("Please wait till home is set")

myinput=1
while 1 :
    # get keyboard input
    myinput = input(">> ")
        # Python 3 users
        # input = input(">> ")
    if myinput == 'exit':
        ser.close()
        exit()
    else:
        # send the character to the device
        # (note that I happend a \n carriage return and line feed to the characters - this is requested by my device)
        myinput = myinput + "\n"
        ser.write(myinput.encode())
        out = b''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1)
            
        if out != '':
            logger.debug(">>" + out.decode())


# G00 X20.00 Y0.00 Z200.00 This means to go to these coordinates as fast as possible
# G01 X20.00 Y0.00 Z200.00 F100 This means to go to these coordinates at the speed designated
# M203 X500.00 Y500.00 Z5.00 E25.00 set maximum feedrates for each thing. 
# M114 might be get the current position of the gantry. 