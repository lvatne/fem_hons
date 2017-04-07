#!/usr/bin/python
import os
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import lights
import kalman
import imageoperations
import time
import cv2

fence = geofence.Geofence()
kalman = kalman.Kalman()
tracker = tracker.Tracker()
cam = camera.Camera()
img = cam.get_picture()
iop = imageoperations.ImageOperations(img, 640, 480)

time.sleep(30) # Give GPS & stuff time to wake up

N_0, E_0, Z_0, Z_1 = kalman.utm_coord()

# Need code to back out from the charging station here ......

while True:
    N_1, E_1, Z_0, Z_1 = kalman.utm_coord()
    while not fence.isinside(N_1, E_1):
        #Turn a random number of degrees, -180 to 180
        delta_hdg = (np.random.ranf() - 0.5) * 360.0
        tracker.turn_relative(delta_hdg)
        tracker.move_dist(0.1)
        N_1, E_1, Z_0, Z_1 = kalman.utm_coord()

    tracker.move_dist(0.3)
    
        
        

