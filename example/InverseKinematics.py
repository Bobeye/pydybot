' Inverse Kinematics Calculation for Robotic Arms with SCARA structure '

__author__ = 'Bowen Weng'

PI = 3.14159265359
VARMG0 = 155
VARM01 = 43
VARM12 = 136
VARM23 = 127
VARM3T = 10

import math


#########Transfer angle in rad to degree########
def RadToDeg(Rad):
	ratio = 180/PI
	Degree = Rad * ratio
	return Degree

#########Calculate 3 angles based on the length of the length of the triangle sides#####
def SidesToAngles(a, b, c):
	A = math.acos( ((b ** 2) + (c ** 2) - (a ** 2)) / (2 * b * c) )
	B = math.acos( ((a ** 2) + (c ** 2) - (b ** 2)) / (2 * a * c) )
	C = math.acos( ((a ** 2) + (b ** 2) - (c ** 2)) / (2 * a * b) )
	Angles = [RadToDeg(A), RadToDeg(B), RadToDeg(C)]
	return Angles

def DescartesToPolar(x, y, z):
	# check if z < 0 before calling the DescartesToPolar() Func
	if x == 0 and y == 0:
		alpha = 0
		belta = 90
	else:
		xy = math.sqrt((x ** 2)+(y ** 2))
		xyz = math.sqrt((x ** 2)+(y ** 2)+(z ** 2))
		alpha = math.acos(x/xy)
		belta = math.acos(xy/xyz)
	alpha = RadToDeg(alpha)
	belta = RadToDeg(belta)
	if y < 0:
		alpha = -alpha
	polar = [alpha, belta, xyz]
	return(polar)

print(DescartesToPolar(-4,-3,0))
	


# if __name__=='__main__':
#     Triangle()