import itertools
import time
import pypot.dynamixel
import math
import random
import matplotlib.pyplot as plt

# Motor configuration
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise IOError('no port found!')
print('ports found', ports)
print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])
Motors = dxl_io.scan(range(10))
print('Motors ID', Motors)


##########Speed Set#####################
for i in Motors:
	dxl_io.set_moving_speed({i: 1000})

LastMotorPos = [0,0,0,0,0,0]
# zero all the motors
def ZEROSERVO():
	for i in Motors:
		dxl_io.set_goal_position({i: 0})
		time.sleep(0.01)
	LastMotorPos = [0,0,0,0,0,0]
	print('Zero all the motors...')


# classify motors
leftMotors = [0]
rightMotors = [0]
for i in Motors:
	if i % 2 == 0:
		leftMotors.append(i)
	else:
		rightMotors.append(i)
leftMotors.pop(0)
rightMotors.pop(0)
print('left set of motors:', leftMotors)
print('right set of motors:', rightMotors)

# Move the servos to desired angle position
def SERVOMOVE(MotorPos,Period):
	permit = 0

	ServoSpeed = range(6)
	MaxSpeed = 360 # the max servo speed is 360 degree/s, coresponding to 360
	SafeSpeedFac = 1.5 # 1.5 is the safe factor for servo speed

	# Speed Settings
	# get current servo position
	CurrentPos = range(6)
	PosDiff = range(6)
	# CurrentInfo = dxl_io.get_present_position_speed_load(Motors)
	for i in range(6):
		CurrentPos[i] = LastMotorPos[i]
		PosDiff[i] = abs(MotorPos[i] - abs(CurrentPos[i]))
		ServoSpeed[i] = PosDiff[i] / Period
		ServoSpeed[i] = ServoSpeed[i]
		if ServoSpeed[i] > SafeSpeedFac * MaxSpeed:
			permit = 1
		if ServoSpeed[i] > MaxSpeed:
			ServoSpeed[i] = MaxSpeed
	for i in Motors:
		dxl_io.set_moving_speed({i: ServoSpeed[i-1]})
	# print('Current sPEED:', ServoSpeed)
	# print(CurrentPos)

	
	for i in MotorPos:
		if abs(i) > 40:
			permit = 2
	if permit == 1:
		print('WARNING!!!!!!! MOTOR SPEED OVER RANGE!!!')
	if permit == 2:
		# print('MOTOR POSITION:', MotorPos)
		print('WARNING!!!!!!! MOTOR OVER RANGE!!!')
	else:
		# print('MOTOR POSITION:', MotorPos)
		dxl_io.set_goal_position({1: -MotorPos[1]})
		dxl_io.set_goal_position({2: MotorPos[2]})
		dxl_io.set_goal_position({3: -MotorPos[3]})
		dxl_io.set_goal_position({4: MotorPos[4]})
		dxl_io.set_goal_position({5: -MotorPos[5]})
		dxl_io.set_goal_position({6: MotorPos[0]})


def ZEROTOPPLATE():
	zeroTop = STEWARTCONTROL([0,0,0,0,0,0])
	SERVOMOVE(zeroTop,0.1)
	time.sleep(2)

def STEWARTCONTROL(StewartControlParameters): # [xdelta, ydelta, zdelta, xangle, yangle, zangle]
	
	# Bottom origin is on the center of the surface of the bottom board;
	ZBottomMotor = 41.5
	ZeroH = 212.0
	LinkA = 75.22
	LinkB = 120.0
	TopAngle = math.atan(float(25.0/70.0))
	BottomAngle = math.atan(float(62.0/102.0))
	TopRadius = float(70.0 * math.acos(TopAngle))
	BottomRadius = float(102.0 * math.acos(BottomAngle))
	# Initial Bottom Coordinates Setup
	BottomCoordinates = [[BottomRadius * math.cos(BottomAngle), -BottomRadius * math.sin(BottomAngle), 0],
						 [BottomRadius * math.cos(BottomAngle), BottomRadius * math.sin(BottomAngle), 0],
						 [-BottomRadius * math.sin(math.radians(30)-BottomAngle), BottomRadius * math.cos(math.radians(30)-BottomAngle), 0],
						 [-BottomRadius * math.sin(math.radians(30)+BottomAngle), BottomRadius * math.cos(math.radians(30)+BottomAngle), 0],
						 [-BottomRadius * math.sin(math.radians(30)+BottomAngle), -BottomRadius * math.cos(math.radians(30)+BottomAngle), 0],
						 [-BottomRadius * math.sin(math.radians(30)-BottomAngle), -BottomRadius * math.cos(math.radians(30)-BottomAngle), 0]]
	# print('BottomCoordinates',BottomCoordinates)
	# Initial Servo Coordinates Setup
	MotorCoordinates = BottomCoordinates
	for i in range(6):
		MotorCoordinates[i][2] = ZBottomMotor
	# print(MotorCoordinates)
	# Initial Top Coordinates Setup
	TopCoordinates = [[TopRadius * math.cos(TopAngle), -TopRadius * math.sin(TopAngle), 0],
						[TopRadius * math.cos(TopAngle), TopRadius * math.sin(TopAngle), 0],
						[-TopRadius * math.sin(math.radians(30)-TopAngle), TopRadius * math.cos(math.radians(30)-TopAngle), 0],
						[-TopRadius * math.sin(math.radians(30)+TopAngle), TopRadius * math.cos(math.radians(30)+TopAngle), 0],
						[-TopRadius * math.sin(math.radians(30)+TopAngle), -TopRadius * math.cos(math.radians(30)+TopAngle), 0],
						[-TopRadius * math.sin(math.radians(30)-TopAngle), -TopRadius * math.cos(math.radians(30)-TopAngle), 0]]
	# print('Initial TOP COORDINATES',TopCoordinates)
	# print(TopCoordinates)
	# Top plate initialization
	for i in range(6):
		TopCoordinates[i][2] = ZeroH

	# top SHIFT
	################Top plate shifting FUNCTION##########
	#Calculate Top plate position with given moving parameters of x, y and z
	def TOPSHIFT(shiftMatrix):	# shiftMatrix is in the format of [x,y,z], standing for the shifting move
		TopShift = TopCoordinates
		for i in range(6):
			TopShift[i][0] = TopShift[i][0] + shiftMatrix[0]
			TopShift[i][1] = TopShift[i][1] + shiftMatrix[1]
			TopShift[i][2] = TopShift[i][2] + shiftMatrix[2]
		return TopShift
	######################################################

	# top ROTATION
	################Top plate rotation FUNCTION##########
	def TOPRATATION(rotationMatrix, InitialTopPlate):
		def QUATERNIONS_ROTATION(vector, axis, angle):
			
			# 4-matrix multiply vector 
			def QUATERNIONS_MULTIPLY(qm0,qv0):
				qm = [0,0,0]
				for i in range(3):
					for j in range(3):
				 		qm[i] = qm[i] + qm0[i][j] * qv0[j]
				return qm
			
			v = 1-math.cos(angle)
			c = math.cos(angle)
			s = math.sin(angle)
			x = axis[0]
			y = axis[1]
			z = axis[2]
			RotQuaternions = [	[1-v*((y*y)+(z*z)) , x*y*v-z*s , x*z*v+y*s] ,
								[x*y*v+z*s , 1-v*((x*x)+(z*z)) , y*z*v-x*s] ,
								[x*z*v-y*s , y*z*v+x*s , 1-v*((x*x)+(y*y))]	]
			return QUATERNIONS_MULTIPLY(RotQuaternions,vector)
		
		for i in range(6):
			InitialTopPlate[i][2] = InitialTopPlate[i][2] - ZeroH
		AfterRotTopPlate = InitialTopPlate
		alpha = rotationMatrix[0]
		belta = rotationMatrix[1]
		gama = rotationMatrix[2]
		if alpha != 0:
			for i in range(6):
				AfterRotTopPlate[i] = QUATERNIONS_ROTATION(InitialTopPlate[i], [1,0,0], alpha)
				InitialTopPlate = AfterRotTopPlate
		if belta != 0:
			for i in range(6):
				AfterRotTopPlate[i] = QUATERNIONS_ROTATION(InitialTopPlate[i], [0,1,0], belta)
				InitialTopPlate = AfterRotTopPlate
		if gama != 0:
			for i in range(6):
				AfterRotTopPlate[i] = QUATERNIONS_ROTATION(InitialTopPlate[i], [0,0,1], gama)
		for i in range(6):
			AfterRotTopPlate[i][2] = AfterRotTopPlate[i][2] + ZeroH
		return AfterRotTopPlate
	######################################################

	###############Aim Topplate Position Calculation############
	#Calculate Servo Position with given aim top plate positions
	############################################################
	def STEWARTCONFIGURE(AimTopPos):
		# print('aim', AimTopPos)
		# Calculate leg length
		LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			TempDistance = 0.0
			for j in range(3):
				TempDistance = TempDistance + ((AimTopPos[i][j]-MotorCoordinates[i][j])**2)
			LegLength[i] = math.sqrt(TempDistance)
		# print('Leg length', LegLength)

		# Calculate leg direction
		LegAngle = AimTopPos
		TempLegAngle = AimTopPos
		for i in range(6):
			for j in range(3):
				LegAngle[i][j] = AimTopPos[i][j] - MotorCoordinates[i][j]
				TempLegAngle[i][j] = LegAngle[i][j]
			LegAngle[i][0], LegAngle[i][1] = LegAngle[i][1], -LegAngle[i][0]
		# print('leg angle', LegAngle)
		# print('temp leg angle', TempLegAngle)

		# MotionPlanning
		# Angle between leg and the leg projector on the surface
		Theta = range(6)
		Gama = [0.0,1.0,2.0,3.0,4.0,5.0]
		Belta = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			Theta[i] = math.atan( LegAngle[i][2] / (math.sqrt((LegAngle[i][0]**2) + (LegAngle[i][1]**2))) )
			Gama[i] = math.acos( ((LinkA**2) + (LegLength[i]**2) - (LinkB**2)) / (2 * LinkA * LegLength[i]) )
			if i <= 1:
				Belta[i] = math.atan(( LegAngle[i][1] / LegAngle[i][0] ))
			elif i == 2:
				axisrot = math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				x1 = x0 * ca + y0 * sa
				y1 = y0 * ca - x0 * sa
				Belta[i] = math.atan(y1/x1)
			elif i == 3:
				axisrot = math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				x1 = x0 * ca + y0 * sa
				y1 = y0 * ca - x0 * sa
				Belta[i] = math.atan(y1/x1)
			elif i == 4:
				axisrot = -math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				x1 = x0 * ca + y0 * sa
				y1 = y0 * ca - x0 * sa
				Belta[i] = math.atan(y1/x1)
			elif i == 5:
				axisrot = -math.pi*2/3
				ca = math.cos(axisrot)
				sa = math.sin(axisrot)
				x0 = LegAngle[i][0]
				y0 = LegAngle[i][1]
				x1 = x0 * ca + y0 * sa
				y1 = y0 * ca - x0 * sa
				Belta[i] = math.atan(y1/x1)
			if Belta[i] > math.pi:
				Belta[i] = Belta[i] % math.pi
		# print('THETA', Theta)
		# print('BELTA', Belta)
		# print('GAMA', Gama)

		# Aim Motor Angle
		AimMotorAngle = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			AimMotorAngle[i] = math.acos( math.cos(math.pi - Theta[i] - Gama[i]) * math.cos(Belta[i]) )
		for i in range(6):
			AimMotorAngle[i] = 90 - math.degrees(AimMotorAngle[i])


		return AimMotorAngle
	##########################################################

	# MAIN
	tempTop = TopCoordinates
	deltaX = StewartControlParameters[0]
	deltaY = StewartControlParameters[1]
	deltaZ = StewartControlParameters[2]
	angleX = StewartControlParameters[3]
	angleY = StewartControlParameters[4]
	angleZ = StewartControlParameters[5]
	if deltaX != 0 or deltaY != 0 or deltaZ != 0:
		tempTop = TOPSHIFT([deltaX, deltaY, deltaZ])
	if angleX != 0 or angleY != 0 or angleZ != 0:
		tempTop = TOPRATATION([angleX, angleY, angleZ], tempTop)
	# print('tempTop Coordinates', tempTop)
	tempmotorPos = STEWARTCONFIGURE(tempTop)

	return tempmotorPos

######################################################################################

# define the movement corresponding to Asin(2*pi*f*T+phi) on 6 DOF range
def SINMOTION(sinMotionMatrix, TimePeriod):  #[[A, f, phi] * 6], Total running time
	SampleTime = 0.01

	A = range(6)
	f = range(6)
	w = range(6)
	phi = range(6)
	T = range(6)
	for i in range(6):
		A[i] = sinMotionMatrix[i][0]	# Amplitude
		f[i] = sinMotionMatrix[i][1]	# Frequency
		phi[i] = sinMotionMatrix[i][2]	# Time shift
		w[i] = 2 * math.pi * f[i]
		T[i] = 1 / f[i]					# Period time

	def SinStartPosition():
		SinStartPosition = range(6)
		for i in range(6):
			SinStartPosition[i] = A[i] * math.sin(phi[i])
		SinStartServo = STEWARTCONTROL(SinStartPosition)
		SERVOMOVE(SinStartServo, 2)
		LastMotorPos = SinStartServo
		print('sin motion ready>>>>>>>>>>>>>>>>>>')
		time.sleep(1)
		print('GO!')

	SinStartPosition()
	CurrentAimPosition = range(6)
	for i in range(6):
		CurrentAimPosition[i] = A[i] * math.sin(phi[i])
	TimeStart = time.time()
	TimeLast = time.time()
	TimePass = 0
	while TimePass <= TimePeriod:
		TimeCurrent = time.time()
		TimeGap = TimeCurrent - TimeLast
		TimePass = TimeCurrent - TimeStart
		if TimeGap >= SampleTime:
			TimeLast = TimeCurrent
			for i in range(6):
				CurrentAimPosition[i] = A[i] * math.sin(w[i] * TimePass + phi[i])
			CurrentAimMotor = STEWARTCONTROL(CurrentAimPosition)
			SERVOMOVE(CurrentAimMotor, TimeLast + SampleTime - time.time())
			LastMotorPos = CurrentAimMotor
		

def RANDOMOCEANWAVEMODEL(timePeriod):
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
		for f in range(1,100):
			f = f/100.0		
			Wmatrix = OceanWave(f,time)
			Wt = Wmatrix[0]
			Wr = Wmatrix[1]
			Wtrans = Wtrans+Wt
			Wrot = Wrot+Wr
		return [Wtrans,Wrot]


	Slist = [0]
	for i in range(1,100):
		i = i/100.0
		S = PMSpectrum(i)
		Slist.append(S)

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
			Wr = W[1]/15.0
			while Wr > math.pi:
				Wr = Wr - math.pi
			while Wr < -math.pi:
				Wr = Wr + math.pi
			if Wt >= 5.0:
				Wt = 5.0
			if Wt <= -5.0:
				Wt = -5.0
			if Wr >= 0.20:
				Wr = 0.20
			if Wr <= -0.20:
				Wr = -0.20
			Wtlist.append(Wt)
			Wrlist.append(Wr)
			print('Trans:', Wt, 'Rot: ', Wr)
			TopWave = [0.0,0.0,Wt,Wr,0.0,0.0]
			CurrentAimMotor = STEWARTCONTROL(TopWave)
			SERVOMOVE(CurrentAimMotor, last + sampletime - current)
			LastMotorPos = CurrentAimMotor
			last = current
		current = time.time()


	# plotting
	plt.figure(1)               # the first figure
	plt.title('Random Wave')

	plt.subplot(311)
	plt.grid(True) 
	plt.ylabel('P-M Wave Spectral Density (m^2/Hz)')
	plt.xlabel('Frequency/0.01Hz')
	dataLine = plt.plot(Slist)
	plt.setp(dataLine, color='g', linewidth=2.0)

	plt.subplot(312)
	plt.grid(True) 
	plt.ylabel('Simulated Wave Height / mm')
	plt.xlabel('SampleNumber')
	flistLine = plt.plot(Wtlist)
	plt.setp(flistLine, color='r', linewidth=2.0)

	plt.subplot(313)
	plt.grid(True) 
	plt.ylabel('Simulated Wave Angle / rad')
	plt.xlabel('SampleNumber')
	flistLine = plt.plot(Wrlist)
	plt.setp(flistLine, color='y', linewidth=2.0)

	plt.show()


def main():
	ZEROTOPPLATE()

	# STABLE--15min
	# 1125-31
	# tempshakeMatrix = [[0.0,1.0,0.0] , [0.0,0.08,(math.pi)/2] , [0.0,0.08,0.0] ,
	# 				   [0.0,0.08,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]





	# 0 direction wave
	# Y translation + X rotation 0.05 Hz
	# 1144-29
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.05,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.22,0.05,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 121-11
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.05,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.17,0.05,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1220-41
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.05,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.12,0.05,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1237-55
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.05,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.07,0.05,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]


	# Y translation + X rotation 0.13 Hz
	# 1254-11
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.13,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.22,0.13,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1316-34
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.13,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.17,0.13,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1334-39
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.13,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.12,0.13,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1350-20
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.13,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.07,0.13,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# Y translation + X rotation 0.2 Hz
	# 146-14
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.2,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.22,0.2,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1422-49
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.2,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.17,0.2,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1440-3
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.2,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.12,0.2,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 150-25
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.2,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.07,0.2,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# Y translation + X rotation 0.3 Hz
	# 1517-22
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.3,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.22,0.3,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1534-56
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.3,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.17,0.3,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1550-38
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.3,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.12,0.3,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 166-30
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.3,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.07,0.3,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# Y translation + X rotation 0.5 Hz
	# 1622-32
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.5,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.22,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1640-18
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.5,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.17,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1656-35
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.5,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.12,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# 1712-45
	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,0.5,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0.07,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]


	# 45 direction wave
	# X translation + Y rotation 0.05 Hz
	# 1728-41
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.17,0.05,(math.pi)/2] , [0.17,0.05,(math.pi)/2] , [0.0,1.0,0] ]

	# 1744-36
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.12,0.05,(math.pi)/2] , [0.12,0.05,(math.pi)/2] , [0.0,1.0,0] ]

	# 180-34
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.07,0.05,(math.pi)/2] , [0.07,0.05,(math.pi)/2] , [0.0,1.0,0] ]

	# X translation + Y rotation 0.5 Hz
	# 2014-12
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.17,0.5,(math.pi)/2] , [0.17,0.5,(math.pi)/2] , [0.0,1.0,0] ]

	# 2030-20
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.12,0.5,(math.pi)/2] , [0.12,0.5,(math.pi)/2] , [0.0,1.0,0] ]

	# 2047-1
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.07,0.5,(math.pi)/2] , [0.07,0.5,(math.pi)/2] , [0.0,1.0,0] ]


	# 90 direction wave
	# X translation + Y rotation 0.05 Hz
	# 213-56
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.05,(math.pi)/2] , [0.17,0.05,(math.pi)/2] , [0.0,1.0,0] ]

	# 2123-21
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.05,(math.pi)/2] , [0.12,0.05,(math.pi)/2] , [0.0,1.0,0] ]

	# 2139-41
	# tempshakeMatrix = [[0.0,0.05,(math.pi)/2] , [0.0,0.05,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.05,(math.pi)/2] , [0.07,0.05,(math.pi)/2] , [0.0,1.0,0] ]



	# X translation + Y rotation 0.5 Hz
	# 2155-13
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.5,(math.pi)/2] , [0.17,0.5,(math.pi)/2] , [0.0,1.0,0] ]

	# 2212-58
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.5,(math.pi)/2] , [0.12,0.5,(math.pi)/2] , [0.0,1.0,0] ]

	# 2228-33
	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,(math.pi)/2] , [0,0.1,0.0] ,
	# 				   [0.0,0.5,(math.pi)/2] , [0.07,0.5,(math.pi)/2] , [0.0,1.0,0] ]



	
	# translation Wave
	# 0.05 Hz
	# 2258-53
	# tempshakeMatrix = [[0.0,0.3,0] , [0.0,0.3,0] , [10.0,0.05,0.0] ,
	# 				   [0.0,0.3,0] , [0.0,0.3,0] , [0.0,0.1,0] ]

	

	# 0.3 Hz
	# 2314-32
	# tempshakeMatrix = [[0.0,0.3,0] , [0.0,0.3,0] , [10.0,0.3,0.0] ,
	# 				   [0.0,0.3,0] , [0.0,0.3,0] , [0.0,0.1,0] ]
	

	# 0.5 Hz
	# 2329-58
	# tempshakeMatrix = [[0.0,0.3,0] , [0.0,0.3,0] , [10.0,0.5,0.0] ,
	# 				   [0.0,0.3,0] , [0.0,0.3,0] , [0.0,0.1,0] ]
	

	# tempshakeMatrix = [[0.0,0.3,0] , [0.0,0.5,0] , [0.0,0.5,0.0] ,
	# 				   [0.0,0.3,0] , [0.0,0.3,0] , [0.15,0.5,0] ]

	# tempshakeMatrix = [[20.0,0.5,0.0] , [0.0,0.5,(math.pi)/2] , [0,0.08,0.0] ,
	# 				   [0,0.08,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]
	# tempshakeMatrix = [[0.0,1.0,0.0] , [9.0,0.15,(math.pi)/2] , [9.6,0.15,0.0] ,
	# 				   [0.12,0.15,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]
	# tempshakeMatrix = [[0.0,1.0,0.0] , [10,0.5,(math.pi)/2] , [5,1.0,0.0] ,
	# 				   [0.2,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# tempshakeMatrix = [[0.0,1.0,0.0] , [10,0.15,(math.pi)/2] , [0,1.0,0.0] ,
	# 				   [0.1,0.15,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# tempshakeMatrix = [[9.0,0.5,(math.pi)/2] , [0.0,1.0,0.0] , [2.6,0.5,0.0] ,
	# 				   [0.0,1.0,0] , [0.03,0.5,(math.pi)/2] , [0.0,1.0,0] ]

	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,1.0,0.0] , [0.0,0.5,0.0] ,
	#  				   [0.0,1.0,0] , [0.0,1.0,0] , [0.3,0.15,0] ]


	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,1.0,0.0] , [12.0,1.0,0.0] ,
	#  				   [0.0,1.0,0] , [0.0,1.0,0] , [0.0,2.0,0] ]

	# tempshakeMatrix = [[0.0,1.0,0.0] , [4.0,0.5,(math.pi)/2] , [16.6,0.5,0.0] ,
	# 				   [0.05,0.5,(math.pi)/2] , [0.0,1.0,0] , [0.0,1.0,0] ]

	# tempshakeMatrix = [[5.0,0.2,(math.pi)/2] , [5.0,0.2,0.0] , [0.0,0.05,0.0] ,
	# 				   [0.1,0.2,(math.pi)/2] , [0.1,0.2,0.0] , [0.0,1.0,0] ]

	# tempshakeMatrix = [[0.0,0.5,0.0] , [0.0,1.0,0.0] , [0.0,4,0.0] ,
	#  				   [0.0,1.0,0] , [0.0,1.0,0] , [0.2,0.5,0] ]

	# tempshakeMatrix = [[0.0,0.5,(math.pi)/2] , [0.0,0.5,0.0] , [0.0,4,0.0] ,
	#  				   [0.1,0.5,0] , [0.1,0.5,(math.pi)/2] , [0.0,1.0,0] ]



	# SINMOTION(tempshakeMatrix, 970)

	RANDOMOCEANWAVEMODEL(910.0)
	ZEROTOPPLATE()
	
	# temp = [0,0,0,0,0,0]
	# print(STEWARTCONTROL(temp))
	# tempStewart = STEWARTCONTROL(temp)
	# SERVOMOVE(tempStewart,0.1)
	# LastMotorPos = tempStewart
	




# tempstewart = STEWARTCONTROL([0,0,0,0,0,0])
# SERVOMOVE(tempstewart,1)
# time.sleep(1)

# for i in range(3):
# 	tempstewart = STEWARTCONTROL([0,0,5,0,0,0])
# 	SERVOMOVE(tempstewart,1)
# 	LastMotorPos = tempstewart
# 	time.sleep(1.001)
# 	tempstewart = STEWARTCONTROL([0,0,-5,0,0,0])
# 	SERVOMOVE(tempstewart,1)
# 	LastMotorPos = tempstewart
# 	time.sleep(1.001)



# ZEROTOPPLATE()
# print('ZEROTOP')
main()


# Stable: 1048-51 0.2021  1232-42 0.2060

#         1215-53 0.1928  