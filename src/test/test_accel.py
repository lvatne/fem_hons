#!/usr/bin/python

import numpy as np
import geofence
import tracker
import motor_sw
import time

g = geofence.Geofence()

m = motor_sw.Motor_sw()
c = tracker.Compass(gauss = 0.88, declination = (0, 52))
c.self_test()

a = tracker.Accelerometer()
time.sleep(3) # Give it some time to stabilize....

#for i in range(100):
#    a.read_raw()
#    time.sleep(0.01)
#exit()

try:
    j = 0
    starttime = time.time() * 1000
    v = np.zeros((1001, 3))
#    for i in range(100):
#        s = a.getAxes()
#        v[j] = s
#        j = j + 1
#        time.sleep(0.01)
#    endtime = time.time() * 1000
#    print("Ran for %d milliseconds" % (endtime - starttime))
#    np.savetxt("accel_data_rest.txt", v)
#    exit()
    m.forward(100)

    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.reverse(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.forward(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.reverse(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.forward(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.reverse(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    time.sleep(0.5)
    m.forward(100)
    m.stop()
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    time.sleep(0.5)
    m.reverse(100)
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)
    m.stop()
    
    for i in range(100):
        s = a.getAxes()
        v[j] = s
        j = j + 1
        time.sleep(0.01)

    np.savetxt("accel_data_fw_bk.txt", v)
except:
    m.stop()
    raise
