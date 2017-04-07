#!/usr/bin/python

import numpy as np
import geofence
import tracker
import motor_sw
import lights
import time

t = tracker.Tracker()
g = t.g
l = lights.Lights()

v = np.zeros((500, 3))
time.sleep(3)
l.headlights(True)
for i in range(500):
    v[i] = g.read()
    time.sleep(0.01)
    
l.headlights(False)
np.savetxt("gyro_data_180.txt", v)
