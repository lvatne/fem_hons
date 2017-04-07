#!/usr/bin/python

import numpy as np
import geofence
import tracker
import motor_sw
import time

t = tracker.Tracker()
t.move_dist(0.4)
t.turn_relative(-20.0)
t.move_dist(0.7)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)
t.turn_relative(40.0)
t.move_dist(0.4)

time.sleep(1)

