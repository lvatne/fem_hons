#!/usr/bin/python

import numpy as np
import geofence
import tracker
import motor_sw
import lights
import time
import sys

# g = geofence.Geofence()
try:
    l = lights.Lights()
    t = tracker.Tracker()
    l.headlights(True)
    
    t.move_dist(0.5)
    time.sleep(1)
    t.turn_relative(90)
    time.sleep(0.5)
    
    t.move_dist(0.5)
    time.sleep(1)
    t.turn_relative(90)
    time.sleep(0.5)
    
    t.move_dist(0.5)
    time.sleep(1)
    t.turn_relative(90)
    time.sleep(0.5)
    
    t.move_dist(0.5)
    time.sleep(1)
    t.turn_relative(90)
    time.sleep(0.5)
    
    l.headlights(False)
    exit()


    
    m = t.m
    c = t.c
    m.forward(70)
    time.sleep(1.5)
    m.steer(-20)  # Turn left (barely)
    time.sleep(1.5)
    m.steer(20)   # Turn right (barely)
    time.sleep(1.5)
    m.stop()
    time.sleep(0.5)
    hdg = c.heading()
    if (hdg > 0):
        hdg = hdg - 180
    elif hdg <= 0:
        hdg = hdg + 180
    t.set_heading(hdg)
    time.sleep(0.5)
    m.forward(70)
    time.sleep(4)
    m.stop()
except:
    print("Exception: ", sys.exc_info()[0])
    raise

# t.move_dist(0.7)
l.headlights(False)
# t.set_heading(90)
# time.sleep(1)
