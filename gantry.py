import time
from serial import Serial
from util import get_logger
import threading

CONFIG = 9

class Gantry:
    def __init__(self):
        self.ser = Serial(
            port='/dev/tty.usbserial-1130',   #port='/dev/ttyUSB0'
            baudrate=115200,
        )
        self.mag = Serial(
            port='/dev/tty.usbmodem187107701',    #'/dev/ttyACM0'
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
        # XMax = 1000
        # ZMax = 500
        XMax = 10
        ZMax = 5
        setMaximumSpeeds = f"M203 X{XMax} Z{ZMax}\n"
        self.ser.write(setMaximumSpeeds.encode())

        self.zero()

        self.current_pos = (0, 0)

        #patterns are named by shape, U is a U, X is an x or cross etc.
        #orientations are numbered starting with the one that looks closest to the label
        #i.e. U: 0 is facing up, X: 0 is an X etc.
        #orientations then rotate clockwise from the starting position
        #for patterns that dont rotate, i.e. vl and hl, they start either to the left or top depending on orientation
        #the 'orientation' then shifts right or down until reaching the end then going back to the middle
        #h_ is horizontal, v_ is vertical, r_ is rotated, m_ is mirrored
        #so hl is horizontal line, mL is mirrored L
        self.patterns = {
            'B': {
                0: [1, 1, 1, 1, 1, 1, 1, 1, 1],
                1: [1, 1, 1, 1, 1, 1, 1, 1, 1],
                2: [1, 1, 1, 1, 1, 1, 1, 1, 1],
                3: [1, 1, 1, 1, 1, 1, 1, 1, 1],
            },
            'U': {
                0: [1, 0, 1, 1, 0, 1, 1, 1, 1],
                1: [1, 1, 1, 1, 0, 0, 1, 1, 1],
                2: [1, 1, 1, 1, 0, 1, 1, 0, 1],
                3: [1, 1, 1, 0, 0, 1, 1, 1, 1],
            },
            'X': {
                0: [1, 0, 1, 0, 1, 0, 1, 0, 1],
                1: [0, 1, 0, 1, 1, 1, 0, 1, 0],
                2: [1, 0, 1, 0, 1, 0, 1, 0, 1],
                3: [0, 1, 0, 1, 1, 1, 0, 1, 0],
            },
            'T': {
                0: [1, 1, 1, 0, 1, 0, 0, 1, 0],
                1: [0, 0, 1, 1, 1, 1, 0, 0, 1],
                2: [0, 1, 0, 0, 1, 0, 1, 1, 1],
                3: [1, 0, 0, 1, 1, 1, 1, 0, 0],
            },
            'O': {
                0: [1, 1, 1, 1, 0, 1, 1, 1, 1],
                1: [1, 1, 1, 1, 0, 1, 1, 1, 1],
                2: [1, 1, 1, 1, 0, 1, 1, 1, 1],
                3: [1, 1, 1, 1, 0, 1, 1, 1, 1],
            },
            'l': {
                0: [0, 1, 0, 0, 1, 0, 0, 1, 0],
                1: [0, 0, 1, 0, .6, 0, 1, 0, 0],
                2: [0, 0, 0, 1, 1, 1, 0, 0, 0],
                3: [1, 0, 0, 0, .6, 0, 0, 0, 1],
            },
            'vl': {
                0: [1, 0, 0, 1, 0, 0, 1, 0, 0],
                1: [0, 1, 0, 0, 1, 0, 0, 1, 0],
                2: [0, 0, 1, 0, 0, 1, 0, 0, 1],
                3: [0, 1, 0, 0, 1, 0, 0, 1, 0],
            },
            'hl': {
                0: [1, 1, 1, 0, 0, 0, 0, 0, 0],
                1: [0, 0, 0, 1, 1, 1, 0, 0, 0],
                2: [0, 0, 0, 0, 0, 0, 1, 1, 1],
                3: [0, 0, 0, 1, 1, 1, 0, 0, 0],
            },
            'L': {
                0: [1, 0, 0, 1, 0, 0, 1, 1, 0],
                1: [1, 1, 1, 1, 0, 0, 0, 0, 0],
                2: [0, 1, 1, 0, 0, 1, 0, 0, 1],
                3: [0, 0, 0, 0, 0, 1, 1, 1, 1],
            },
            'mL': {
                0: [0, 0, 1, 0, 0, 1, 0, 1, 1],
                1: [0, 0, 0, 1, 0, 0, 1, 1, 1],
                2: [1, 1, 0, 1, 0, 0, 1, 0, 0],
                3: [1, 1, 1, 0, 0, 1, 0, 0, 0],
            },
            'J': {
                0: [0, 0, 1, 1, 0, 1, 1, 1, 1],
                1: [1, 1, 0, 1, 0, 0, 1, 1, 1],
                2: [1, 1, 1, 1, 0, 1, 1, 0, 0],
                3: [1, 1, 1, 0, 0, 1, 0, 1, 1],
            },
            'mJ': {
                0: [1, 0, 0, 1, 0, 1, 1, 1, 1],
                1: [1, 1, 1, 1, 0, 0, 1, 1, 0],
                2: [1, 1, 1, 1, 0, 1, 0, 0, 1],
                3: [0, 1, 1, 0, 0, 1, 1, 1, 1],
            },
        }
    
    def activate_mag(self, magnet, attraction, duty):
        msg = bytearray([CONFIG, magnet, 0 if attraction else 1, duty, 255])
        self.logger.debug(msg.hex(' '))
        self.mag.write(msg)

    #set magnets into a predefined pattern
    def mag_pattern(self, pattern, orientation, amplitude):
        if pattern in self.patterns:
            states = self.patterns[pattern][orientation]
            for i, duty in enumerate(states, start=1):
                self.activate_mag(i, True, int(255*duty*amplitude))
    
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