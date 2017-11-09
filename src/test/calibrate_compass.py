#!/usr/bin/python

import numpy as np
import math
import tracker
import motor_sw
import sysprops
import time
import logging

m = motor_sw.Motor_sw()
t = tracker.Tracker()
c = tracker.Compass(gauss = 0.88, declination = (0, 52))


raw = np.zeros((250, 3), float)
m.left_fwd(100)
for i in range(250):
    raw[i] = c.axes()
    time.sleep(0.066667)

m.stop()
    
# print(raw)

np.savetxt("cal_array.txt", raw)

x_min = np.amin(raw[:,0])
x_max = np.amax(raw[:,0])
y_min = np.amin(raw[:,1])
y_max = np.amax(raw[:,1])
x_offset = np.average(raw[:,0])
y_offset = np.average(raw[:,1])

# np.savetxt("x_val.txt", raw[:,0])
# np.savetxt("y_val.txt", raw[:,1])

x_range = math.fabs(x_min) + math.fabs(x_max)
y_range = math.fabs(y_min) + math.fabs(y_max)

xy_ratio = x_range / y_range

print("x_min %4f x_max %4f y_min %4f y_max %4f" % (x_min, x_max, y_min, y_max))
print("x_range %4f y_range %4f xy_ratio %4f" % (x_range, y_range, xy_ratio))
print("x_offset %4f y_offset %4f" % (x_offset, y_offset))
logger = logging.getLogger('navigation')
logger.info("Compass calibrated xy_ratio %4f x_offset %4f y_offset %4f" % (xy_ratio, x_offset, y_offset))
logger.info("x_min %4f x_max %4f y_min %4f y_max %4f" % (x_min, x_max, y_min, y_max))

s = sysprops.SysProps()
s.set('x_offset', x_offset)
s.set('y_offset', y_offset)
s.set('xy_ratio', xy_ratio)

s.store()


