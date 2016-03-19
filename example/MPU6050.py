import itertools
import time
import numpy as np
import matplotlib.pyplot as plt
import random
import serial
ardu = serial.Serial('/dev/ttyUSB0', 57600)
a = ardu.readline()
alist = [a]
yaw = [0]
pitch = [0]
roll = [0]
# ansplist = [0]
# ampyplist = [0]
# sump = 0
# R = 33000
period = 0

print(a)

time.sleep(3)
ardu.write('s')

start = time.time()
print("timer start")



try:
	while(True):
		a = ardu.readline()
		alist.append(a)
		print("Collecting Data")
		print("Press Ctrl-C to Quit")
		print(a)

except KeyboardInterrupt:
	
	# print(alist)

	end = time.time()
	print(end - start)
	period = end - start

	data = [0]
	for i in range(len(alist)):
		# data.append(alist[i].strip('\n'))
		data.append(alist[i].strip("\r\n"))
	data.pop(0)
	data.pop(0)


	print(data)

	for i in range(len(data)-2):
		try:
			data[i] = float(data[i])
		except ValueError,e:
			data[i] = 10000
			# print("error",e,"on item",i)

	for i in range(len(data)-2):
		if abs(data[i]-1000.0) <= 360:
			yaw.append(data[i]-1000.0)
		if abs(data[i]-2000.0) <= 360:
			pitch.append(data[i]-2000.0)
		if abs(data[i]-3000.0) <= 360:
			roll.append(data[i]-3000.0)
	yaw.pop(0)
	pitch.pop(0)
	roll.pop(0)
	print(yaw)
	print(pitch)
	print(roll)


	print("Working time:", period, "s")

	print("Sampling Frequency:", len(yaw)/period)

	# plotting
	plt.figure(1)               # the first figure
	plt.title('VIBRATION TEST')

	plt.subplot(311)
	plt.grid(True) 
	plt.ylabel('yaw')
	dataLine = plt.plot(yaw)
	plt.setp(dataLine, color='g', linewidth=2.0)

	plt.subplot(312)
	plt.grid(True) 
	plt.ylabel('pitch')
	flistLine = plt.plot(pitch)
	plt.setp(flistLine, color='r', linewidth=2.0)

	plt.subplot(313)
	plt.grid(True) 
	plt.ylabel('roll')
	flistLine = plt.plot(roll)
	plt.setp(flistLine, color='y', linewidth=2.0)

	plt.show()

	