import itertools
import time
import pypot.dynamixel
import math

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

# zero all the motors
for i in Motors:
	dxl_io.set_goal_position({i: 0})
	time.sleep(0.01)
LastMotorPos = [0,0,0,0,0,0]
print('Zero all the motors...')
time.sleep(1)

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
	ServoSpeed = range(6)
	MaxSpeed = 360 # the max servo speed is 360 degree/s, coresponding to 360

	# Speed Settings
	# get current servo position
	CurrentPos = range(6)
	PosDiff = range(6)
	# CurrentInfo = dxl_io.get_present_position_speed_load(Motors)
	for i in range(6):
		CurrentPos[i] = LastMotorPos[i]
		PosDiff[i] = abs(MotorPos[i] - abs(CurrentPos[i]))
		ServoSpeed[i] = PosDiff[i] // Period
		ServoSpeed[i] = ServoSpeed[i] + 1
	for i in Motors:
		dxl_io.set_moving_speed({i: ServoSpeed[i-1]})
	print('Current sPEED:', ServoSpeed)
	print(CurrentPos)

	permit = 0
	for i in MotorPos:
		if abs(i) > 40:
			permit = 0
		else:
			permit = 1
	if permit == 0:
		print('WARNING!!!!!!! MOTOR OVER RANGE!!!')
	else:
		print('MOTOR POSITION:', MotorPos)
		dxl_io.set_goal_position({1: -MotorPos[1]})
		dxl_io.set_goal_position({2: MotorPos[2]})
		dxl_io.set_goal_position({3: -MotorPos[3]})
		dxl_io.set_goal_position({4: MotorPos[4]})
		dxl_io.set_goal_position({5: -MotorPos[5]})
		dxl_io.set_goal_position({6: MotorPos[0]})

def STEWARTCONTROL(StewartControlParameters): # [xdelta, ydelta, zdelta, xangle, yangle, zangle]
	
	# Bottom origin is on the center of the surface of the bottom board;
	ZBottomMotor = 41.5
	ZeroH = 210.0
	LinkA = 75.22
	LinkB = 121.0
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
	# print(BottomCoordinates)
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
	print('Initial TOP COORDINATES',TopCoordinates)
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
		# Calculate leg length
		LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			TempDistance = 0.0
			for j in range(3):
				TempDistance = TempDistance + ((AimTopPos[i][j]-MotorCoordinates[i][j])**2)
			LegLength[i] = math.sqrt(TempDistance)
		print('Leg length', LegLength)

		# Calculate leg direction
		LegAngle = AimTopPos
		for i in range(6):
			 for j in range(3):
			 	LegAngle[i][j] = AimTopPos[i][j] - MotorCoordinates[i][j]

		# MotionPlanning
		# Angle between leg and the leg projector on the surface
		Theta = range(6)
		Gama = [0.0,1.0,2.0,3.0,4.0,5.0]
		Belta = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			Theta[i] = math.atan( LegAngle[i][2] / (math.sqrt((LegAngle[i][0]**2) + (LegAngle[i][1]**2))) )
			Gama[i] = math.acos( ((LinkA**2) + (LegLength[i]**2) - (LinkB**2)) / (2 * LinkA * LegLength[i]) )
			if i <= 1:
				Belta[i] = math.atan(abs( LegAngle[i][0] / LegAngle[i][1] ))
			elif i == 2 | i == 5:
				Belta[i] = math.atan(abs( LegAngle[i][0] / LegAngle[i][1] )) -math.radians(30)
			else:
				Belta[i] = math.atan(abs( LegAngle[i][0] / LegAngle[i][1] )) -math.radians(60)
		print('THETA', Theta)
		print('BELTA', Belta)
		print('GAMA', Gama)

		# Aim Motor Angle
		AimMotorAngle = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			AimMotorAngle[i] = math.acos( math.cos(math.pi - Theta[i] - Gama[i]) * math.cos(Belta[i]) )
		for i in range(6):
			AimMotorAngle[i] = 90 - math.degrees(AimMotorAngle[i])

		return AimMotorAngle
	##########################################################

	# MAIN
	deltaX = StewartControlParameters[0]
	deltaY = StewartControlParameters[1]
	deltaZ = StewartControlParameters[2]
	angleX = StewartControlParameters[3]
	angleY = StewartControlParameters[4]
	angleZ = StewartControlParameters[5]
	tempTop = TOPSHIFT([deltaX, deltaY, deltaZ])
	tempTop = TOPRATATION([angleX, angleY, angleZ], tempTop)
	print('tempTop Coordinates', tempTop)
	tempmotorPos = STEWARTCONFIGURE(tempTop)

	return tempmotorPos

######################################################################################


tempstewart = STEWARTCONTROL([0,0,0,0,0,0])
SERVOMOVE(tempstewart,1)
time.sleep(1)
for i in range(3):
	tempstewart = STEWARTCONTROL([0,0,5,0,0,0])
	SERVOMOVE(tempstewart,1)
	LastMotorPos = tempstewart
	time.sleep(1.001)
	tempstewart = STEWARTCONTROL([0,0,-5,0,0,0])
	SERVOMOVE(tempstewart,1)
	LastMotorPos = tempstewart
	time.sleep(1.001)


