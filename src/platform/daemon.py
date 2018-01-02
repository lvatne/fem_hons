#!/usr/bin/python
import os
import sys
import time
import sysprops
import RPi.GPIO as GPIO
import logging
import tracker
import motor_sw
import camera
import lights

def run_square():
    try:
        l = lights.Lights()
        t = tracker.Tracker()
        cam = camera.Camera()
        c = t.c

        l.headlights(True)
        time.sleep(0.2)
        l.headlights(False)
    
        cam.take_picture()
        cam.take_nav_picture()
        # print("Heading %f" % (c.heading()))
        c.test_read()

        t.move_dist(0.5)
        time.sleep(1)
        t.turn_relative(90)
        time.sleep(0.5)
        # print("Heading %f" % (c.heading()))
        c.test_read()
    
        cam.take_picture()
        cam.take_nav_picture()
        t.move_dist(0.5)
        time.sleep(1)
        t.turn_relative(90)
        time.sleep(0.5)
        # print("Heading %f" % (c.heading()))
        c.test_read()
    
        cam.take_picture()
        cam.take_nav_picture()
        t.move_dist(0.5)
        time.sleep(1)
        t.turn_relative(90)
        time.sleep(0.5)
        # print("Heading %f" % (c.heading()))
        c.test_read()
    
        cam.take_picture()
        cam.take_nav_picture()
        t.move_dist(0.5)
        time.sleep(1)
        t.turn_relative(90)
        time.sleep(0.5)
        # print("Heading %f" % (c.heading()))
        c.test_read()
    
        l.headlights(False)
    except:
        print("Exception: ", sys.exc_info()[0])
        raise


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.IN)
    time.sleep(10)
    run_mode = False
    debug_mode = False

    run_mode = GPIO.input(4)
    if run_mode == 1:
        debug_mode = False
    else:
        debug_mode = True

    if run_mode == 1:
        run_square()
        
    exit()        
