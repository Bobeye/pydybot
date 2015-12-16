' Serial communication with Arduino '

__author__ = 'Bowen Weng'

import serial
arduino = serial.Serial('/dev/tty.usbxxx, 9600')
while True:
	print arduino.readline()=