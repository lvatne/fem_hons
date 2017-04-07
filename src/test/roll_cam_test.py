#!/usr/bin/python
import os
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import lights
import time

time.sleep(60)
cam = camera.Camera()

cam.take_picture()

t = tracker.Tracker()

t.move_dist(0.7)
cam.take_picture()
time.sleep(1)
t.turn_relative(10)
cam.take_picture()
time.sleep(1)
t.move_dist(0.4)
cam.take_picture()
t.turn_relative(-10)
time.sleep(1)
cam.take_picture()
t.move_dist(0.4)
time.sleep(1)
cam.take_picture()
t.move_dist(0.4)
time.sleep(1)
cam.take_picture()
t.move_dist(0.4)
time.sleep(1)
cam.take_picture()

exit()
