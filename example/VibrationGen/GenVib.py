import itertools
import time
import pypot.dynamixel
import numpy as np
import matplotlib.pyplot as plt
import random
import serial
ardu = serial.Serial('/dev/ttyUSB2', 115200)
a = ardu.readline()
alist = [a]
flist = [0]
slist = [0]

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

Motors = dxl_io.scan(range(10))

SpeedRate = 0.666
Period = 0.1
QuaPeriod = Period / 4
Amplitude = 45

dxl_io.set_goal_position({1: 0})
time.sleep(3)

print "Press Enter to quit..."
while len(alist)<=30:
	Distance = random.randint(15,Amplitude)
	AngleSpeed = Distance * 4 // Period
	MotorSpeed = AngleSpeed // SpeedRate
	if MotorSpeed > 1023:
		MotorSpeed = 1023
	dxl_io.set_moving_speed({1: MotorSpeed})
	for i in list([1,0,-1,0]):
		dxl_io.set_goal_position({1: Distance*i})
		time.sleep(QuaPeriod + 0.07)
	print("CurrentDistance:", Distance)
	print("CurrentSpeed:", MotorSpeed)
	flist.append(Distance)
	slist.append(AngleSpeed)
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
slist.pop(0)

for i in range(0, len(data)):
	data[i] = float(data[i]) / 1023 * 5

print(data)
print(flist)
print(slist)

plt.figure(1)               # the first figure
plt.title('ANSWENERGY VIBRATION TEST')

plt.subplot(311)
plt.grid(True) 
plt.ylabel('Speed')
slistLine = plt.plot(slist)
plt.setp(slistLine, color='b', linewidth=2.0)

plt.subplot(312)
plt.grid(True) 
plt.ylabel('Amplitude')
flistLine = plt.plot(flist)
plt.setp(flistLine, color='r', linewidth=2.0)

plt.subplot(313)
plt.grid(True) 
plt.ylabel('Voltage')
dataLine = plt.plot(data)
plt.setp(dataLine, color='g', linewidth=2.0)
plt.show()