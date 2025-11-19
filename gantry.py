import time
from serial import Serial
from util import get_logger

class Gantry:
    def __init__(self):
        self.ser = Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
        )
        self.logger = get_logger()
        while not self.ser.isOpen():
            time.sleep(1)

        time.sleep(3) # Give the printer a chance to get ready to receive messages
        self.clearSerial()

        self.logger.info('Initializing gantry system.')
        XMax = 1000
        ZMax = 500
        setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
        self.ser.write(setMaximumSpeeds.encode())

        goHome = "G28\n"
        self.ser.write(goHome.encode())
        self.logger.info("Please wait till home is set")

        self.waitForResponse()
        self.logger.info("Home is set")

        self.current_pos = (0, 0)

    def clearSerial(self):
        while self.ser.inWaiting() > 0:
            self.ser.readline()

    def waitForResponse(self):
        getLoc = "M114\n"
        self.ser.write(getLoc.encode())
        self.logger.debug(f"Waiting for response")

        while self.ser.inWaiting() > 0:
            out = self.ser.readline()
            if out != '':
                self.logger.debug(out.decode())
                if "Count" in out.decode():
                    # position = out.decode.split(' ')[1:]
                    # Get the first and third positions since x and z are what we care about
                    # self.current_pos = (int(position[0][1:]), int(position[2][1:]))
                    break
        

    def send_gcode(self, gcode):
        gcode = gcode + "\n"
        self.logger.debug(f"Sending Command: {gcode}")
        self.ser.write(gcode.encode())

    def go_to_position(self, x, y):
        gcode = f"G00 X{x} Z{y}"
        self.send_gcode(gcode)
        self.waitForResponse()

    def go_to_position_at_speed(self, x, y, speed):
        gcode = f"G01 X{x} Z{y} F{speed}"
        self.send_gcode(gcode)
        self.waitForResponse()