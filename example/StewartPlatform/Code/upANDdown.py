import itertools
import time
import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])
dxl_io = pypot.dynamixel.DxlIO(ports[0])

Motors = dxl_io.scan(range(10))

print(Motors)

# for i in Motors:
# 	dxl_io.set_moving_speed({i: 100})

# zero all the motors
for i in Motors:
	dxl_io.set_goal_position({i: 0})
	time.sleep(0.01)
print('Zero all the motors...')
time.sleep(2)

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
time.sleep(1)	

# up & down function
def updown(pos):
	currentPos = pos
	for i in leftMotors:
		dxl_io.set_goal_position({i: currentPos})
	for i in rightMotors:
		dxl_io.set_goal_position({i: -currentPos})


currentPos = 20
for i in leftMotors:
	dxl_io.set_goal_position({i: currentPos})
for i in rightMotors:
	dxl_io.set_goal_position({i: -currentPos})

for i in list(range(30)):
	updown(38)
	time.sleep(0.5)
	updown(-8)
	time.sleep(0.5)



