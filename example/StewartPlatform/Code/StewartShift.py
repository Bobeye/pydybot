import itertools
import time
import pypot.dynamixel
import math

def StewartMove(goal):
	# Motor configuration
	ports = pypot.dynamixel.get_available_ports()
	if not ports:
	    raise IOError('no port found!')
	# print('ports found', ports)
	# print('connecting on the first available port:', ports[0])
	dxl_io = pypot.dynamixel.DxlIO(ports[0])
	Motors = dxl_io.scan(range(10))
	# print('Motors ID', Motors)


	##########Speed Set#####################
	# for i in Motors:
	# 	dxl_io.set_moving_speed({i: 100})

	# zero all the motors
	# for i in Motors:
	# 	dxl_io.set_goal_position({i: 0})
	# 	time.sleep(0.01)
	# print('Zero all the motors...')
	# time.sleep(2)

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
	# print('left set of motors:', leftMotors)
	# print('right set of motors:', rightMotors)

	# Move the servos to desired angle position
	def ServoMove(MotorPos):
		dxl_io.set_goal_position({1: -MotorPos[1]})
		dxl_io.set_goal_position({2: MotorPos[2]})
		dxl_io.set_goal_position({3: -MotorPos[3]})
		dxl_io.set_goal_position({4: MotorPos[4]})
		dxl_io.set_goal_position({5: -MotorPos[5]})
		dxl_io.set_goal_position({6: MotorPos[0]})



	###################################Motion Planning#####################################
	# Bottom origin is on the center of the surface of the bottom board;
	ZBottomMotor = 41.5
	ZeroH = 210.0
	LinkA = 75.22
	LinkB = 121.0
	TopAngle = math.atan(float(25.0/70.0))
	BottomAngle = math.atan(float(62.0/102.0))
	TopRadius = float(70.0 * math.acos(TopAngle))
	BottomRadius = float(102.0 * math.acos(BottomAngle))
	# print('BA', BottomAngle)
	# Initial Bottom Coordinates Setup
	BottomCoordinates = [[-BottomRadius * math.sin(BottomAngle), -BottomRadius * math.cos(BottomAngle), 0],
						 [BottomRadius * math.sin(BottomAngle), -BottomRadius * math.cos(BottomAngle), 0],
						 [BottomRadius * math.cos(math.radians(30)-BottomAngle), BottomRadius * math.sin(math.radians(30)-BottomAngle), 0],
						 [BottomRadius * math.cos(math.radians(30)+BottomAngle), BottomRadius * math.sin(math.radians(30)+BottomAngle), 0],
						 [-BottomRadius * math.cos(math.radians(30)+BottomAngle), BottomRadius * math.sin(math.radians(30)+BottomAngle), 0],
						 [-BottomRadius * math.cos(math.radians(30)-BottomAngle), BottomRadius * math.sin(math.radians(30)-BottomAngle), 0]]
	# print(BottomCoordinates)
	# Initial Servo Coordinates Setup
	MotorCoordinates = BottomCoordinates
	for i in range(6):
		MotorCoordinates[i][2] = ZBottomMotor
	# print(MotorCoordinates)
	# Initial Top Coordinates Setup
	TopCoordinates = [[-TopRadius * math.sin(TopAngle), -TopRadius * math.cos(TopAngle), 0],
						[TopRadius * math.sin(TopAngle), -TopRadius * math.cos(TopAngle), 0],
						[TopRadius * math.cos(math.radians(30)-TopAngle), TopRadius * math.sin(math.radians(30)-TopAngle), 0],
						[TopRadius * math.cos(math.radians(30)+TopAngle), TopRadius * math.sin(math.radians(30)+TopAngle), 0],
						[-TopRadius * math.cos(math.radians(30)+TopAngle), TopRadius * math.sin(math.radians(30)+TopAngle), 0],
						[-TopRadius * math.cos(math.radians(30)-TopAngle), TopRadius * math.sin(math.radians(30)-TopAngle), 0]]
	# print(TopCoordinates)
	# Top plate initialization
	for i in range(6):
		TopCoordinates[i][2] = 210

	# top SHIFT
	################Top plate shifting FUNCTION##########
	#Calculate Top plate position with given moving parameters of x, y and z
	def TopShift(shiftMatrix):
		for i in range(6):
			TopCoordinates[i][2] = 210
		topShift = TopCoordinates
		for i in range(6):
			topShift[i][0] = topShift[i][0] + shiftMatrix[0]
			topShift[i][1] = topShift[i][1] + shiftMatrix[1]
			topShift[i][2] = topShift[i][2] + shiftMatrix[2]
		return topShift
	######################################################


	# top ROTATION

	def TopRotX(alpha):
		TopRot = TopCoordinates
		TopRotX = TopCoordinates
		for i in range(6):
			TopRotX[i][1] = TopRot[i][1] * math.cos(alpha)
			TopRotX[i][2] = TopRot[i][2] + (TopRot[i][1] * math.sin(alpha))
		return TopRotX

	def TopRotY(belta):
		TopRot = TopCoordinates
		TopRotY = TopCoordinates
		for i in range(6):
			TopRotY[i][0] = TopRot[i][0] * math.cos(-belta)
			TopRotY[i][2] = TopRot[i][2] + (TopRot[i][0] * math.sin(-belta))
		return TopRotY

	######################################################



	###############Aim Topplate Position Calculation############
	#Calculate Servo Position with given aim top plate positions
	############################################################
	def StewartConfigure(AimTopPos):
		# Calculate leg length
		LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			TempDistance = 0.0
			for j in range(3):
				TempDistance = TempDistance + ((AimTopPos[i][j]-MotorCoordinates[i][j])**2)
			LegLength[i] = math.sqrt(TempDistance)
		print('LL', LegLength)

		# Calculate leg direction
		LegAngle = AimTopPos
		for i in range(6):
			 for j in range(3):
			 	LegAngle[i][j] = AimTopPos[i][j] - MotorCoordinates[i][j]
		# print('legangle',LegAngle)

		# MotionPlanning
		# Angle between leg and the leg projector on the surface
		Theta = range(6)
		Gama = [0.0,1.0,2.0,3.0,4.0,5.0]
		Belta = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			Theta[i] = math.atan( LegAngle[i][2] / (math.sqrt((LegAngle[i][0]**2) + (LegAngle[i][1]**2))) )
			Gama[i] = math.acos( ((LinkA*LinkA) + (LegLength[i]*LegLength[i]) - (LinkB*LinkB)) / (2 * LinkA * LegLength[i]) )
			if i <= 1:
				Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] ))
			elif i == 2 | i == 5:
				Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(30)
			else:
				Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(60)
		# print('Theta',Theta)
		# print('Gama',Gama)
		# print('Belta',Belta)

		# Aim Motor Angle
		AimMotorAngle = [0.0,1.0,2.0,3.0,4.0,5.0]
		for i in range(6):
			AimMotorAngle[i] = math.acos( math.cos(math.pi - Theta[i] - Gama[i]) * math.cos(Belta[i]) )
		for i in range(6):
			AimMotorAngle[i] = 90 - math.degrees(AimMotorAngle[i])

		return AimMotorAngle
	###################################################################################


	########################################Temp
	# CurrentTopPlate = TopShift([15,15,10])
	# print(StewartConfigure(CurrentTopPlate))
	# rot = TopRotation(-0.1, 0.0, 0.0)
	# print('rot', rot)
	#########################################END OF MOTION PLANNING##################################################


	# def goto(goalPos):
	# 	TempTopPlate = TopShift(goalPos)
	# 	print(TempTopPlate)
	# 	TempPos = StewartConfigure(TempTopPlate)
	# 	print('MotorPosition: ', TempPos)
	# 	ServoMove(TempPos)
	# 	time.sleep(0.1)

	def goto(goalPos):
		if math.radians(goalPos[0]) != 0:
			xRotAngle = math.radians(goalPos[0])
			TempTopPlate = TopRotX(xRotAngle)
		else:
			yRotAngle = math.radians(goalPos[1])
			TempTopPlate = TopRotY(yRotAngle)
		TempPos = StewartConfigure(TempTopPlate)
		print('MotorPosition: ', TempPos)
		ServoMove(TempPos)
		time.sleep(0.1)

	goto(goal)

	# up & down function
	# def shift(pos):
	# 	currentPos = pos
	# 	for i in leftMotors:
	# 		dxl_io.set_goal_position({i: currentPos})
	# 	for i in rightMotors:
	# 		dxl_io.set_goal_position({i: -currentPos})


	# currentPos = 0
	# for i in leftMotors:
	# 	dxl_io.set_goal_position({i: currentPos})
	# for i in rightMotors:
	# 	dxl_io.set_goal_position({i: -currentPos})

	# for i in list(range(30)):
	# 	updown(20)
	# 	time.sleep(0.2)
	# 	updown(-8)
	# 	time.sleep(0.2)

StewartMove([0,0,0])
time.sleep(1)
for i in range(10):
	StewartMove([20,0,0])
	# StewartMove([0,0.3,0])
	# time.sleep(0.01)
	StewartMove([-20,0,0])
	# time.sleep(0.01)
	# StewartMove([0,-0.3,0])
	# time.sleep(0.01)
# for i in range(5):
# 	StewartMove([-7,0,0])
# 	StewartMove([7,0,0])
StewartMove([0,0,0])