import random
import math
import matplotlib.pyplot as plt
import time

# def RANDOMWAVE(TimePeriod):
def PMSpectrum(freq):
	Wspeed = 8
	w = 2 * math.pi * freq
	Spm = 0.81 / (w**5) * math.exp(-0.74 * ((9.8/Wspeed/w)**4))
	return Spm

def OceanWave(freq,time):
	Sw = PMSpectrum(freq)
	w = 2 * math.pi * freq
	Atrans = math.sqrt(2 * Sw * w)
	Arot = (w*w/9.8)*Atrans
	phi = random.randrange(0,62800,1) / 10000.0
	Wtrans = Atrans * math.cos(w*time + phi)
	Wrot = Arot * math.cos(w*time + phi)
	return [Wtrans,Wrot]

def RandomOceanWave(time):
	Wtrans = 0
	Wrot = 0
	for f in range(5,1000):
		f = f/100.0		
		Wmatrix = OceanWave(f,time)
		Wt = Wmatrix[0]
		Wr = Wmatrix[1]
		Wtrans = Wtrans+Wt
		Wrot = Wrot+Wr
	return [Wtrans,Wrot]

	# start = time.time()
	# current = time.time()
	# while current-start <= TimePeriod:		
	# 	randomTransA = random.randrange(0,10,1)
	# 	randomRotA = random.randrange(0,0.2,0.01)
	# 	randomTransf = random.randrange(0,1.5,0.1)
	# 	randomRotf = random.randrange(0,1.5,0.01)
	# 	randomTransPhi = random.randrange(-math.pi,math.pi,0.1)
	# 	randomRotPhi = random.randrange(-math.pi,math.pi,0.1)
	# 	RandomWaveA = randomTransA * math.sin(randomTransf * current + randomTransPhi)

	# 	current = time.time()

def main():
	Slist = [0]
	for i in range(1,100):
		i = i/100.0
		S = PMSpectrum(i)
		Slist.append(S)

	timePeriod = 300.0
	sampletime = 0.1
	start = time.time()
	last = time.time()
	current = time.time()
	Wtlist = [0]
	Wrlist = [0]
	while current-start <= timePeriod:
		if current-last >= sampletime:
			W = RandomOceanWave(current)
			Wt = W[0]
			Wr = W[1]/25.0
			if Wt >= 10.0:
				Wt = 10.0
			if Wt <= -10.0:
				Wt = -10.0
			if Wr >= 0.22:
				Wr = 0.22
			if Wr <= -0.22:
				Wr = -0.22
			Wtlist.append(Wt)
			Wrlist.append(Wr)
			print('Trans:', Wt, 'Rot: ', Wr)
			last = current
		current = time.time()


	# plotting
	plt.figure(1)               # the first figure
	plt.title('Random Wave')

	plt.subplot(311)
	plt.grid(True) 
	plt.ylabel('Wave Energy')
	plt.xlabel('Frequency')
	dataLine = plt.plot(Slist)
	plt.setp(dataLine, color='g', linewidth=2.0)

	plt.subplot(312)
	plt.grid(True) 
	plt.ylabel('At')
	flistLine = plt.plot(Wtlist)
	plt.setp(flistLine, color='r', linewidth=2.0)

	plt.subplot(313)
	plt.grid(True) 
	plt.ylabel('Ar')
	flistLine = plt.plot(Wrlist)
	plt.setp(flistLine, color='y', linewidth=2.0)

	plt.show()


main()
