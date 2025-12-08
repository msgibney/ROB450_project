import time
from serial import Serial
from util import get_logger
import threading

CONFIG = 9

class Gantry:
    def __init__(self):
        self.ser = Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
        )
        self.mag = Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
        )
        self.logger = get_logger()
        while not self.ser.isOpen():
            time.sleep(1)
        while not self.mag.isOpen():
            time.sleep(1)

        time.sleep(3) # Give the printer a chance to get ready to receive messages
        self.clearSerial()
        for i in range(1, 10):
            self.activate_mag(i, True, 0)
        self.activate_mag(5, True, 225)

        self.read_mag_thread = threading.Thread(target=self.read_mag)
        self.read_mag_thread.daemon = True
        self.read_mag_thread.start()

        self.logger.info('Initializing gantry system.')
        XMax = 1000
        ZMax = 500
        setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
        self.ser.write(setMaximumSpeeds.encode())

        self.zero()

        self.current_pos = (0, 0)
    
    def activate_mag(self, magnet, attraction, duty):
        msg = bytearray([CONFIG, magnet, 0 if attraction else 1, duty, 255])
        self.logger.debug(msg.hex(' '))
        self.mag.write(msg)
    
    def read_mag(self):
        while True:
            while self.mag.inWaiting() > 0:
                self.logger.debug(self.mag.readline())

    def clearSerial(self):
        while self.ser.inWaiting() > 0:
            self.ser.readline()
    
    def get_curr_pos(self):
        return self.current_pos

    def zero(self):
        goHome = "G28\n"
        self.ser.write(goHome.encode())
        self.logger.info("Please wait till home is set")

        time.sleep(3)
        self.clearSerial()

        self.waitForResponse()
        self.logger.info("Home is set")
        

    def waitForResponse(self):
        getLoc = "M114\n"
        self.ser.write(getLoc.encode())
        self.logger.debug(f"Waiting for response")
        gotResponse = False

        while not gotResponse:
            while self.ser.inWaiting() > 0:
                out = self.ser.readline()
                if out != '':
                    if "busy" in out.decode():
                        continue
                    self.logger.debug(f"Response: {out.decode()}")
                    if "Count" in out.decode():
                        gotResponse = True
                        # position = out.decode().split(' ')
                        # # Get the first and third positions since x and z are what we care about
                        # self.current_pos = (float(position[0][2:]), float(position[2][2:]))
                        # self.logger.debug(f"At current position {self.current_pos}")
                        break
        

    def send_gcode(self, gcode):
        gcode = gcode + "\n"
        self.logger.debug(f"Sending Command: {gcode}")
        self.ser.write(gcode.encode())

    def go_to_position(self, x, y):
        x = min(x, 430)
        y = min(y, 300)
        gcode = f"G00 X{y} Z{x}"
        self.current_pos = (float(x), float(y))
        self.send_gcode(gcode)
        self.waitForResponse()

    def go_to_position_at_speed(self, x, y, speed):
        gcode = f"G01 X{x} Z{y} F{speed}"
        self.send_gcode(gcode)
        self.waitForResponse()