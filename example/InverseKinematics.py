' Inverse Kinematics Calculation for Robotic Arms with SCARA structure '

__author__ = 'Bowen Weng'

PI = 3.14159265359
VARMG0 = 155
VARM01 = 43
VARM12 = 136
VARM23 = 127
VARM3T = 0
VJOINT0 = [-120, 120]
VJOINT1 = [0, 90]
VJOINT2 = [-90, 90]
VJOINT3 = [-90, 90]


import math

#########Calculate 3 angles based on the length of the length of the triangle sides#####
def SidesToAngles(a, b, c):
	A = math.acos( ((b ** 2) + (c ** 2) - (a ** 2)) / (2 * b * c) )
	B = math.acos( ((a ** 2) + (c ** 2) - (b ** 2)) / (2 * a * c) )
	C = math.acos( ((a ** 2) + (b ** 2) - (c ** 2)) / (2 * a * b) )
	Angles = [A, B, C]
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
	if y < 0:
		alpha = -alpha
	polar = [alpha, belta, xyz]
	return(polar)

def TempPolar(polar):
	temppolar = [polar[0], 0 ,0]



print(DescartesToPolar(-4,-3,0))
	


# if __name__=='__main__':
#     Triangle()