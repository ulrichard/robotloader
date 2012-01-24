#!/usr/bin/python
from serial import *
import time
import sys
import termios

def output_data(filedev, ignore = '\r'):
    '''Outputs data from serial port to sys.stdout.'''

    while True:
        byte = filedev.read(1)
        if not byte:
            break
        if byte in ignore: continue
        sys.stdout.write(byte)

if __name__ == "__main__":
	# Open device with RTS pin low
	port = Serial('/dev/ttyUSB0', 38400, timeout=0.5)

	# reset the robot arm
	port.flushInput()
	port.setRTS(True)
	time.sleep(0.3) 
	port.setRTS(False)

	time.sleep(1.0)

	port.flushInput()
	port.write("\xff\xff\x00\x00\x00\x00\xff")

	try:
		while 1 :
		    x = port.read(1)
		    sys.stdout.write(x)

	except KeyboardInterrupt:
		print 'Keyboard interrupt, closing.'

	port.close()
	print '--- Done ---'




