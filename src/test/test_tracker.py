#!/usr/bin/python

import numpy as np
import geofence
import tracker
import motor_sw
import time

t = tracker.Tracker()

print("X os % 3.2f Y os %3.2f Z os %3.2f" % (t.x_offset, t.y_offset, t.z_offset))
for i in range(20):
    t.get_inertial_measurements()
    print(t.inertial_data)

# t.move_dist(0.5)
# t.turn_relative(-20.0)
# t.move_dist(0.7)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)
# t.turn_relative(40.0)
# t.move_dist(0.4)

#  time.sleep(1)

