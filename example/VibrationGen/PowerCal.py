import itertools
import time
import pypot.dynamixel
import numpy as np
import matplotlib.pyplot as plt
import random
import serial
ardu = serial.Serial('/dev/ttyUSB0', 115200)
a = ardu.readline()
alist = [a]
plist = [0]
sump = 0
R = 10
period = 0

start = time.time()
print("timer start")



try:
	while(True):
		a = ardu.readline()
		plist = [0]
		alist.append(a)
		print("Collecting Data")
		print("Press Ctrl-C to Quit")
		if a >= 0:
			print("data received......")

except KeyboardInterrupt:
	print('\n')

	end = time.time()
	print(end - start)
	period = end - start


	# print(alist)

	data = [0]
	for i in range(len(alist)):
		data.append(alist[i].strip('\n'))
	data.pop(0)
	data.pop(0)


	# voltage calculation
	for i in range(0, len(data)):
		data[i] = float(data[i]) / 1023 * 5
	# print(data)

	# power calculation
	for i in range(0, len(data)):
		power = data[i] * data[i] / R
		plist.append(power)
	plist.pop(0)

	# Final result
	for i in range(0, len(plist)):
		sump = sump + plist[i]
	AvePower = (sump / len(plist)) * 1000
	print("Average power is:", AvePower, "mW")
	print("Working time:", period, "s")
	Energy = AvePower * period
	print("total energy:", Energy, 'mJ')


	# plotting
	plt.figure(1)               # the first figure
	plt.title('ANSWENERGY VIBRATION TEST')

	plt.subplot(211)
	plt.grid(True) 
	plt.ylabel('Voltage')
	dataLine = plt.plot(data)
	plt.setp(dataLine, color='g', linewidth=2.0)

	plt.subplot(212)
	plt.grid(True) 
	plt.ylabel('Power')
	flistLine = plt.plot(plist)
	plt.setp(flistLine, color='r', linewidth=2.0)

	plt.show()