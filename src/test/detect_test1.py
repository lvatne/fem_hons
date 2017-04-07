#!/usr/bin/python
import os
import camera
import numpy as np
import geofence
import tracker
import motor_sw
import lights
import imageoperations as imop
import time
import cv2

cam = camera.Camera()

img = cam.get_picture()
iop = imop.ImageOperations(img, 640, 480)

dst = iop.correct_foreshortening(45)

keyps = iop.blobdetect_RGB()

n = len(keyps)
print("Found %d keypoints" % (n))

max = 0
idx = -1
for i in range(n):
    if keyps[i].size > max:
        max = keyps[i].size
        idx = i

print("Largest keypoint: %d " % (idx))

if idx > -1:
    angle, distance = iop.movement_vector(keyps[idx])
    print("Move A: %f Dist: %f" % (angle, distance))

im_with_keypoints = cv2.drawKeypoints(dst, keyps, np.array([]),
                                      (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

iop.archive_image(im_with_keypoints)
# Show blobs
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
cv2.destroyAllWindows()
