#!/usr/bin/python
import os
import sys
import cv2
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import lights
import imageoperations as imop
import time


t = tracker.Tracker()

time.sleep(2)
cam = camera.Camera()

for i in range(20):
    img = cam.get_picture()
    iop = imop.ImageOperations(img, 640, 480)
    keyps = iop.blobdetect_HSV()
    im_with_keypoints = cv2.drawKeypoints(img, keyps, np.array([]),
                                          (0,0,255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    iop.archive_image(im_with_keypoints)
    # cam.take_picture()
    t.move_dist(0.2)
    time.sleep(0.1)

t.turn_relative(90)
t.move_dist(0.1)
t.turn_relative(90)

for i in range(20):
    img = cam.get_picture()
    iop = imop.ImageOperations(img, 640, 480)
    keyps = iop.blobdetect_HSV()
    im_with_keypoints = cv2.drawKeypoints(img, keyps, np.array([]),
                                          (0,0,255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    iop.archive_image(im_with_keypoints)
    # cam.take_picture()
    t.move_dist(0.2)
    time.sleep(0.1)
    
t.turn_relative(90)
t.move_dist(0.1)
t.turn_relative(90)


sys.exit()

t.move_dist(0.1)
cam.take_picture()
time.sleep(1)
t.turn_relative(10)
cam.take_picture()
time.sleep(1)
t.move_dist(0.1)
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
