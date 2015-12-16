' Serial communication with Arduino '

__author__ = 'Bowen Weng'

import serial
arduino = serial.Serial('/dev/tty.usbxxx, 9600')
ardata = [0]
while True:
	print arduino.readline()
	ardata.append(arduino.readline())
ardata.pop(0)
print(ardata)