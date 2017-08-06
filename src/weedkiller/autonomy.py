#!/usr/bin/python
import os
import sys
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import lights
import kalman
import imageoperations
import time
import math
import cv2
import sysprops

kalman = kalman.Kalman()
tracker = tracker.Tracker()
cam = camera.Camera()
img = cam.get_picture()
iop = imageoperations.ImageOperations(img, 640, 480)

time.sleep(3) # Give GPS & stuff time to wake up

N_0, E_0, Z_0, Z_1 = kalman.utm_coord()
fence = geofence.Geofence(N_0, E_0)

print(fence.perimeter)
s = sysprops.SysProps()
# s.store()
print(s.collect_PID_data)
# sys.exit()

# Need code to back out from the charging station here ......
for i in range(10):
    kalman.test()
    time.sleep(1)
    N_1, E_1, Z_0, Z_1 = kalman.utm_coord()
    print("N_1 %f E_1 %f" % (N_1, E_1))
    if N_1 > -997.0 and N_1 < -999.0:
        kalman.reset()
        time.sleep(2)



step_distance = 0.15

# while True:
for i in range(30):
    N_1, E_1, Z_0, Z_1 = kalman.utm_coord()
    hdg = tracker.c.heading()
    rad_hdg = tracker.c.heading_rad()

    nextN = N_1 + step_distance * math.cos(rad_hdg)
    nextE = E_1 + step_distance * math.sin(rad_hdg)

    attempts = 0
    while not fence.isinside(nextE, nextN):
        x, y = fence.getRelXY(nextE, nextN)
        print("NOT inside: hdg: %f X: %f Y:%f, rel X: %f rel Y %f" % (hdg, nextE, nextN, x, y))
        #Turn a random number of degrees, -180 to 180
        delta_hdg = (np.random.ranf() - 0.5) * 360.0
        tracker.turn_relative(delta_hdg)
        # tracker.move_dist(0.1)
        N_1, E_1, Z_0, Z_1 = kalman.utm_coord()
        hdg = tracker.c.heading()
        rad_hdg = tracker.c.heading_rad()

        nextN = N_1 + step_distance * math.cos(rad_hdg)
        nextE = E_1 + step_distance * math.sin(rad_hdg)
        attempts = attempts + 1
        if attempts > 10:
            break

    x, y = fence.getRelXY(E_1, N_1)
    print("inside: hdg:%f X:%f Y:%f, rel X:%f rel Y%f" % (hdg, E_1, N_1, x, y))

    tracker.move_dist(step_distance)
    img = cam.get_picture()
    iop.set_image(img, 640, 480)
    iop.archive_image(img)
    
    
    
        
        

