import itertools
import time
import numpy as np
import matplotlib.pyplot as plt
import random
import serial
ardu = serial.Serial('/dev/ttyUSB2', 115200)
a = ardu.readline()
alist = [a]
ansplist = [0]
ampyplist = [0]
sump = 0
R = 33000
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
	for i in range(len(alist)-2):
		data.append(alist[i].strip('\n'))
	data.pop(0)
	data.pop(0)

	ans = [0]
	ampy = [0]
	for i in range(len(data)-2):
		if data[i] == 'ans':
			ans.append(data[i+1])
		if data[i] == 'ampy':
			ampy.append(data[i+1])
	ans.pop(0)
	ampy.pop(0)
	print(ans)
	print(ampy)


	# voltage calculation
	for i in range(0, len(ans)):
		ans[i] = float(ans[i]) / 1023 * 5
	for i in range(0, len(ampy)):
		ampy[i] = float(ampy[i]) / 1023 * 5
	print(ans)
	print(ampy)

	# power calculation
	for i in range(0, len(ans)):
		anspower = ans[i] * ans[i] / R
		ansplist.append(anspower)
	ansplist.pop(0)
	for i in range(0, len(ampy)):
		ampypower = ampy[i] * ampy[i] / R
		ampyplist.append(ampypower)
	ampyplist.pop(0)

	# Final result
	sump = 0
	for i in range(0, len(ansplist)):
		sump = sump + ansplist[i]
	ansAvePower = (sump / len(ansplist)) * 100000
	print("18650 Average power is:", ansAvePower, "mW")
	sump = 0
	for i in range(0, len(ampyplist)):
		sump = sump + ampyplist[i]
	ampyAvePower = (sump / len(ampyplist)) * 1000
	print("Ampy Average power is:", ampyAvePower, "mW")



	# print("Working time:", period, "s")
	# Energy = AvePower * period
	# print("total energy:", Energy, 'mJ')


	# # plotting
	# plt.figure(1)               # the first figure
	# plt.title('ANSWENERGY VIBRATION TEST')

	# plt.subplot(211)
	# plt.grid(True) 
	# plt.ylabel('Voltage')
	# dataLine = plt.plot(data)
	# plt.setp(dataLine, color='g', linewidth=2.0)

	# plt.subplot(212)
	# plt.grid(True) 
	# plt.ylabel('Power')
	# flistLine = plt.plot(plist)
	# plt.setp(flistLine, color='r', linewidth=2.0)

	# plt.show()