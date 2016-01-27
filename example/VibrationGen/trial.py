import time

try:
    while(True):
        time.sleep(1)
except KeyboardInterrupt:
	sum = 0
	for i in range(100):
		sum = sum + i
	print('\n')
	print(sum)
	print "KeyboardInterrupt detected!"