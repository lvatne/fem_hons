#!/usr/bin/python

import numpy as np
import smbus
import math
import time
import sys
import os
import sysprops
import tracker

t = tracker.Tracker()
t3 = 0

for i in range(500):
    t1 = time.time()
#    acc = t.a.getAxes()
#        self.acc_buf[idx] = acc[0] - self.x_offset # Apply calibration
#    gyr = t.g.read()
#        self.tstamp[idx] = time.time()
#        instant_gyro = gyr[2] # It's the Z rotation we're using
    hdg = t.c.heading()
    t2 = time.time()
    t3 = t3 + (t2 - t1)
 
print("Avg read time %f" % (t3 / 500))

