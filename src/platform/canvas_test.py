#!/usr/bin/python

import numpy as np
import geofence
#import tracker
#import motor_sw
import time
from tkinter import *

def cx(x):
    x_adj = 10
    return 2 * (x + x_adj)
def cy(y):
    y_adj = 200
    return 2 * (y * -1 + y_adj)
    

master = Tk()


w = Canvas(master, width=500, height=500)
w.pack()

g = geofence.Geofence()
# m = motor_sw.Motor_sw()
# m.stop()
# exit()
# g.save()


n = len(g.geofence)

print(g.isinside(100.123, 100.09))
w.create_oval(cx(99.123), cy(99.09), cx(101.123), cy(101.09), fill="green", width=1)

print(g.isinside(-100, -100))
print(g.isinside(50, 50))
n = len(g.geofence)
for i in range(n-1):
    w.create_line(cx(g.geofence[i][0]), cy(g.geofence[i][1]), cx(g.geofence[i+1][0]), cy(g.geofence[i+1][1]), fill="green", width=3)

# w.create_rectangle(50, 20, 150, 80, fill="#476042")
# w.create_rectangle(65, 35, 135, 65, fill="yellow")
# w.create_line(0, 0, 50, 20, fill="#476042", width=3)
# w.create_line(0, 100, 50, 80, fill="#476042", width=3)
# w.create_line(150,20, 200, 0, fill="#476042", width=3)
# w.create_line(150, 80, 200, 100, fill="#476042", width=3)

# m = motor_sw.Motor_sw()
# c = tracker.Compass(gauss = 0.88, declination = (0, 52))
# c.self_test()

# a = tracker.Accelerometer()

try:
    print( a.getAxes(True) )
    #print("X: %4d Y: %4d Z: %4d" % (x, y, z))
    for i in range(5):
        (x, y, z) = a.getAxes(True)
        print("X: %4f Y: %4f Z: %4f" % (x, y, z))
        time.sleep(0.1)
    m.left_fwd(50)
    m.right_fwd(50)
    for i in range(5):
        (x, y, z) = a.getAxes(True)
        print("X: %4f Y: %4f Z: %4f" % (x, y, z))
        time.sleep(0.1)
    m.stop()
    m.right_rev(100)
    m.left_rev(100)
    for i in range(5):
        (x, y, z) = a.getAxes(True)
        print("X: %4f Y: %4f Z: %4f" % (x, y, z))
        time.sleep(0.1)
    m.stop()
    for i in range(5):
        (x, y, z) = a.getAxes(True)
        print("X: %4f Y: %4f Z: %4f" % (x, y, z))
        time.sleep(0.5)
    exit()
except:
    m.stop()
    raise

t = tracker.Tracker()

t.set_heading(70)
print(t.c.local_heading_deg())
time.sleep(3)
t.set_heading(-70)
print(t.c.local_heading_deg())
time.sleep(3)
t.set_heading(10)
print(t.c.local_heading_deg())
time.sleep(3)
t.set_heading(-180)
print(t.c.local_heading_deg())
time.sleep(3)
t.set_heading(0)
print(t.c.local_heading_deg())


exit()

# c = tracker.Compass(gauss = 0.88, declination = (0, 52))
# c.self_test()
# c.reset()
# while True:
#     c.test_read()
#     time.sleep(1)

while True:
        (x, y, z) = c.axes()
        h = c.degrees(c.heading())
        hdg = h[0]
        hdg += 90
        if hdg > 180:
            tmp = hdg - 180
            hdg = -180 + tmp
        if hdg < -180:
            tmp = hdg + 180
            hdg = 180 - tmp

        hdg = c.local_heading_deg()
        print("H: %5d X:%5d Y:%5d Z:%5d " % (hdg, x, y, z))
        #sys.stdout.write("\rHeading: " + str(c.degrees(c.heading())) + " " + x + y + z)
        #sys.stdout.flush()
        time.sleep(1)
print("starting motor")
m.left_fwd(100)

for i in range(10):
        (x, y, z) = c.axes()
        hdg = c.degrees(c.heading())
        
        print("H: %3d X:%5f Y:%5f Z:%5f " % (hdg[0], x, y, z))
        #sys.stdout.write("\rHeading: " + str(c.degrees(c.heading())) + " " + x + y + z)
        #sys.stdout.flush()
        time.sleep(1)

m.stop()
        
for i in range(30000):
    w.create_oval(cx(t.cur_x-1), cy(t.cur_y-1), cx(t.cur_x+1), cy(t.cur_y+1), fill="#476042", width=1)
    t.advance(1)
    w.create_oval(cx(t.cur_x-1), cy(t.cur_y-1), cx(t.cur_x+1), cy(t.cur_y+1), fill="yellow", width=1)
    
    np = t.nextpoint(8)
    if (not g.isinside(np[0], np[1])):
        t.newheading()
        np = t.nextpoint(8)
        if (not g.isinside(np[0], np[1])):
            t.newheading()
            np = t.nextpoint(8)
            if (not g.isinside(np[0], np[1])):
                t.newheading()
                np = t.nextpoint(8)
                if (not g.isinside(np[0], np[1])):
                    # Start methodical resolve
                    print("New heading: going around")
                    curh = t.cur_hdg
                    for d in range(180):
                        if curh < 0:
                            t.cur_hdg = curh + d
                        else:
                            t.cur_hdg = curh - d
                        np = t.nextpoint(8)
                        if (g.isinside(np[0], np[1])):
                            print("On ", d, "th attempt ", t.cur_hdg)
                            break
                    # No resolution yet
                    if (not g.isinside(np[0], np[1])):
                        if curh < 0:
                            curh += 180
                        else:
                            curh -= 180
                        for d in range(180):
                            if curh < 0:
                                t.cur_hdg = curh + d
                            else:
                                t.cur_hdg = curh - d
                            np = t.nextpoint(8)
                            if (g.isinside(np[0], np[1])):
                                print("On ", d + 180, "th attempt ", t.cur_hdg)
                                break
                            

    Tk.update_idletasks(w)
    Tk.update(w)
    time.sleep(0.001)

