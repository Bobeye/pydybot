' Serial communication with Arduino '

__author__ = 'Bowen Weng'

import serial
ardu = serial.Serial('/dev/ttyUSB1', 115200)

a = ardu.readline()
alist = [a]

while len(alist)<=100:
	a = ardu.readline()
	alist.append(a)
	print(a)

data = [0]
for i in range(len(alist)):
    data.append(alist[i].strip('\n'))
data.pop(0)
data.pop(0)

print(data)