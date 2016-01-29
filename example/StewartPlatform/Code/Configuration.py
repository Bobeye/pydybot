import math

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
print('BottomCoordinates',BottomCoordinates)
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
print('TC',TopCoordinates)
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

	# Calculate leg direction
	LegAngle = AimTopPos
	for i in range(6):
		 for j in range(3):
		 	LegAngle[i][j] = AimTopPos[i][j] - MotorCoordinates[i][j]
	print('legangle',LegAngle)

	# MotionPlanning
	# Angle between leg and the leg projector on the surface
	Theta = range(6)
	Gama = [0.0,1.0,2.0,3.0,4.0,5.0]
	Belta = [0.0,1.0,2.0,3.0,4.0,5.0]
	for i in range(6):
		Theta[i] = math.atan( LegAngle[i][2] / (math.sqrt((LegAngle[i][0]**2) + (LegAngle[i][1]**2))) )
		Gama[i] = math.acos( ((LinkA**2) + (LegLength[i]**2) - (LinkB**2)) / (2 * LinkA * LegLength[i]) )
		if i <= 1:
			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] ))
		elif i == 2 | i == 5:
			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(30)
		else:
			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(60)

	# Aim Motor Angle
	AimMotorAngle = [0.0,1.0,2.0,3.0,4.0,5.0]
	for i in range(6):
		AimMotorAngle[i] = math.acos( math.cos(math.pi - Theta[i] - Gama[i]) * math.cos(Belta[i]) )
	for i in range(6):
		AimMotorAngle[i] = 90 - math.degrees(AimMotorAngle[i])

	return AimMotorAngle



###################################################################################









# import math

# # Bottom origin is on the center of the surface of the bottom board;
# # 
# ZBottomMotor = 41.5
# ZeroH = 210.0
# LinkA = 75.22
# LinkB = 121.0
# TopAngle = math.atan(float(25.0/70.0))
# BottomAngle = math.atan(float(62.0/102.0))
# TopRadius = float(70.0 * math.acos(TopAngle))
# BottomRadius = float(102.0 * math.acos(BottomAngle))
# print('BA', BottomAngle)

# # Initial Bottom Coordinates Setup
# BottomCoordinates = [[-BottomRadius * math.sin(BottomAngle), -BottomRadius * math.cos(BottomAngle), 0],
# 					 [BottomRadius * math.sin(BottomAngle), -BottomRadius * math.cos(BottomAngle), 0],
# 					 [BottomRadius * math.cos(math.radians(30)-BottomAngle), BottomRadius * math.sin(math.radians(30)-BottomAngle), 0],
# 					 [BottomRadius * math.cos(math.radians(30)+BottomAngle), BottomRadius * math.sin(math.radians(30)+BottomAngle), 0],
# 					 [-BottomRadius * math.cos(math.radians(30)+BottomAngle), BottomRadius * math.sin(math.radians(30)+BottomAngle), 0],
# 					 [-BottomRadius * math.cos(math.radians(30)-BottomAngle), BottomRadius * math.sin(math.radians(30)-BottomAngle), 0]]
# print(BottomCoordinates)

# # Initial Servo Coordinates Setup
# MotorCoordinates = BottomCoordinates
# for i in range(6):
# 	MotorCoordinates[i][2] = ZBottomMotor
# print(MotorCoordinates)

# # Initial Top Coordinates Setup
# TopCoordinates = [[-TopRadius * math.sin(TopAngle), -TopRadius * math.cos(TopAngle), 0],
# 					[TopRadius * math.sin(TopAngle), -TopRadius * math.cos(TopAngle), 0],
# 					[TopRadius * math.cos(math.radians(30)-TopAngle), TopRadius * math.sin(math.radians(30)-TopAngle), 0],
# 					[TopRadius * math.cos(math.radians(30)+TopAngle), TopRadius * math.sin(math.radians(30)+TopAngle), 0],
# 					[-TopRadius * math.cos(math.radians(30)+TopAngle), TopRadius * math.sin(math.radians(30)+TopAngle), 0],
# 					[-TopRadius * math.cos(math.radians(30)-TopAngle), TopRadius * math.sin(math.radians(30)-TopAngle), 0]]
# print(TopCoordinates)

# # Top plate initialization
# for i in range(6):
# 	TopCoordinates[i][2] = 210


# # top SHIFT
# ################Top plate shifting FUNCTION##########
# #Calculate Top plate position with given moving parameters of x, y and z
# def TopShift(shiftMatrix):
# 	TopShift = TopCoordinates
# 	for i in range(6):
# 		TopShift[i][0] = TopShift[i][0] + shiftMatrix[0]
# 		TopShift[i][1] = TopShift[i][1] + shiftMatrix[1]
# 		TopShift[i][2] = TopShift[i][2] + shiftMatrix[2]
# 	return TopShift
# ######################################################





# # top ROTATION
# ################Top plate rotation FUNCTION##########
# #Calculate Top plate position with given angle parameters of phi, belta and gama
# # def MultiRot(C, M):
# # 	MultiRot = range(3)
# # 	for i in range(3):
# # 		for j in range(3):
# # 			MultiRot[i] = MultiRot[i] + C[j] * M[j][i]
# # 	return MultiRot

# # def TopRotation(Phi, Belta, Alpha):
# # 	TopRotation = TopCoordinates
# # 	Rx = [[1.0, 0.0, 0.0] , [0.0, math.cos(Phi), -math.sin(Phi)] , [0.0, math.sin(Phi), math.cos(Phi)]]
# # 	Ry = [[math.cos(Belta), 0.0, math.sin(Belta)] , [0.0, 1.0, 0.0] , [-math.sin(Belta), 0.0, math.cos(Belta)]]
# # 	Rz = [[math.cos(Alpha), -math.sin(Alpha), 0.0] , [math.sin(Alpha), math.cos(Alpha), 0.0] , [0.0, 0.0, 1.0]]
# # 	for i in range(6):
# # 		TopRotation[i] = MultiRot(TopRotation[i], Rz)
# # 		TopRotation[i] = MultiRTopot(T(alpha):
# # 			Rotation[i], Ry)
# # 		TopRotation[i] = MultiRot(TopRotation[i], Rz)
# # 	return TopRotation
# #########################################################################################################


# def TopRotX(alpha):
# 	TopRot = TopCoordinates
# 	TopRotX = TopCoordinates
# 	for i in range(6):
# 		TopRotX[i][1] = TopRot[i][1] * math.cos(alpha)
# 		TopRotX[i][2] = TopRot[i][2] + TopRot[i][1] * math.sin(alpha)
# 	return TopRotX







# ###############Aim Topplate Position Calculation############
# #Calculate Servo Position with given aim top plate positions
# ############################################################
# def StewartConfigure(AimTopPos):
# 	# Calculate leg length
# 	LegLength = [0.0,1.0,2.0,3.0,4.0,5.0]
# 	for i in range(6):
# 		TempDistance = 0.0
# 		for j in range(3):
# 			TempDistance = TempDistance + ((AimTopPos[i][j]-MotorCoordinates[i][j])**2)
# 		LegLength[i] = math.sqrt(TempDistance)

# 	# Calculate leg direction
# 	LegAngle = AimTopPos
# 	for i in range(6):
# 		 for j in range(3):
# 		 	LegAngle[i][j] = AimTopPos[i][j] - MotorCoordinates[i][j]
# 	print('legangle',LegAngle)

# 	# MotionPlanning
# 	# Angle between leg and the leg projector on the surface
# 	Theta = range(6)
# 	Gama = [0.0,1.0,2.0,3.0,4.0,5.0]
# 	Belta = [0.0,1.0,2.0,3.0,4.0,5.0]
# 	for i in range(6):
# 		Theta[i] = math.atan( LegAngle[i][2] / (math.sqrt((LegAngle[i][0]**2) + (LegAngle[i][1]**2))) )
# 		Gama[i] = math.acos( ((LinkA**2) + (LegLength[i]**2) - (LinkB**2)) / (2 * LinkA * LegLength[i]) )
# 		if i <= 1:
# 			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] ))
# 		elif i == 2 | i == 5:
# 			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(30)
# 		else:
# 			Belta[i] = math.atan(abs( LegAngle[i][1] / LegAngle[i][0] )) -math.radians(60)
# 	print('Theta',Theta)
# 	print('Gama',Gama)
# 	print('Belta',Belta)

# 	# Aim Motor Angle
# 	AimMotorAngle = [0.0,1.0,2.0,3.0,4.0,5.0]
# 	for i in range(6):
# 		AimMotorAngle[i] = math.acos( math.cos(math.pi - Theta[i] - Gama[i]) * math.cos(Belta[i]) )
# 	for i in range(6):
# 		AimMotorAngle[i] = 90 - math.degrees(AimMotorAngle[i])

# 	return AimMotorAngle
# ###################################################################################


# ########################################

# ALPHA = math.radians(10)
# CurrentTopPlate = TopRotX(ALPHA)
# print('CTP', CurrentTopPlate)
# print(StewartConfigure(CurrentTopPlate))