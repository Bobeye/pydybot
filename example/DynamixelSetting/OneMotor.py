import itertools
import time
import pypot.dynamixel
import numpy as np
import matplotlib.pyplot as plt
import random
import serial
ardu = serial.Serial('/dev/ttyUSB1', 115200)
a = ardu.readline()
alist = [a]
flist = [0]

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

Motors = dxl_io.scan(range(10))

SpeedRate = 0.666
Period = 0.2
QuaPeriod = Period / 4
Amplitude = 12

dxl_io.set_goal_position({1: 0})
time.sleep(5)

print "Press Enter to quit..."
while len(alist)<=10:
	Distance = random.randint(0,Amplitude)
	AngleSpeed = Distance * 4 // Period
	MotorSpeed = AngleSpeed // SpeedRate
	if MotorSpeed > 1023:
		MotorSpeed = 1023
	dxl_io.set_moving_speed({1: MotorSpeed})
	for i in list([1,0,-1,0]):
		dxl_io.set_goal_position({1: Distance*i})
		time.sleep(QuaPeriod + 0.01)
	print("CurrentDistance:", Distance)
	print("CurrentSpeed:", MotorSpeed)
	flist.append(Distance)
	###########################
	a = ardu.readline()
	alist.append(a)
	print("CurrentVol:", a)

data = [0]
for i in range(len(alist)):
    data.append(alist[i].strip('\n'))
data.pop(0)
data.pop(0)
flist.pop(0)

print(data)
print(flist)

plt.plot(flist)
plt.plot(data)
plt.show()
