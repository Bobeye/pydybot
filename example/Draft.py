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

for i in Motors:
	dxl_io.set_moving_speed({i: 100})

dxl_io.set_goal_position({1: 0})
dxl_io.set_goal_position({3: 90})
dxl_io.set_goal_position({5: 90})
dxl_io.set_goal_position({7: 90})

result = dxl_io.get_present_position_speed_load(Motors)
print(result)

# for position in list(range(-90,90)):
# 	dxl_io.set_goal_position({1: position})
# 	time.sleep(0.01)
# 	dxl_io.set_goal_position({3: position})
# 	time.sleep(0.01)




# speed = dxl_io.get_moving_speed([1,3])
# print(speed)
# time.sleep(5)
# result = dxl_io.get_present_position_speed_load([1,3])
# print(result)


# for position in list(range(-120,120)):
# 	dxl_io.set_goal_position({1: position})
# 	time.sleep(0.01)
# 	dxl_io.set_goal_position({7: position})
# 	time.sleep(0.01)

# dxl_io.set_joint_mode([1])


# print(dxl_io.get_control_mode([1]))