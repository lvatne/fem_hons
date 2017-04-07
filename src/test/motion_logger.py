#!/usr/bin/python
import os
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import kalman
import lights
import time

time.sleep(60)

t = tracker.Tracker()
k = kalman.Kalman()

t.move_dist(0.7)
t.dump_buf("tracker_1.txt")
k.dump_buf("gps1.txt")
time.sleep(1)
t.turn_relative(10)
t.dump_buf("tracker_2.txt")
k.dump_buf("gps2.txt")
time.sleep(1)
t.move_dist(0.4)
t.dump_buf("tracker_3.txt")
k.dump_buf("gps3.txt")
time.sleep(1)
t.turn_relative(-10)
t.dump_buf("tracker_4.txt")
k.dump_buf("gps4.txt")
time.sleep(1)
t.move_dist(0.4)
t.dump_buf("tracker_5.txt")
k.dump_buf("gps5.txt")
time.sleep(1)
t.move_dist(0.4)
t.dump_buf("tracker_6.txt")
k.dump_buf("gps6.txt")
time.sleep(1)

t.move_dist(0.4)
time.sleep(1)


exit()
