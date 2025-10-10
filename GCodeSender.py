import time
from serial import Serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

ser.isOpen()

print('Enter your commands below.\r\nInsert "exit" to leave the application.')

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
        # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        myinput = myinput + "\n"
        ser.write(myinput.encode())
        out = b''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            out += ser.read(1)
            
        if out != '':
            print(">>" + out.decode())


# G00 X20.00 Y0.00 Z200.00 This means to go to these coordinates as fast as possible
# G01 X20.00 Y0.00 Z200.00 F100 This means to go to these coordinates at the speed designated