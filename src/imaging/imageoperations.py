#!/usr/bin/python


import os
import math
import time
import numpy as np
import cv2
import logging
import sysprops

class ImageOperations:

    def __init__(self, img, width, height):
        self.set_image(img, width, height)
        props = sysprops.SysProps()
        self.imgdir = os.path.join(props.robohome, "img")
        self.logger = logging.getLogger('camera_img')   
        

    def set_image(self, img, width, height):
        self.img = img
        self.alt_img = img
        self.orig_width = width
        self.orig_height = height
        self.new_width = width
        self.new_height = height
        self.bottom_x_offset = 0

    def get_image(self):
        return self.img
        
    # Correct foreshortening like a top-hinged window swinging away
    # from the observer

    def correct_foreshortening(self, degrees):
        """
    Correct foreshortening; like a top-hinged window swinging away
    from the observer
    Keyword arguments:
    img -- the image to be corrected
    degrees -- number of degrees to swing the image. 0 to 90 degrees
    width -- width of the original- and resulting image
    height -- height of the original- and resulting image
    """
        self.new_height = math.floor(self.orig_height * math.cos(math.radians(degrees)))
        self.new_width = math.floor(self.orig_width * math.cos(math.radians(degrees)))
        # Corners, clockwise from top left
        pts1 = np.float32([[0,0],
                           [self.orig_width,0],
                           [self.orig_width, self.orig_height],
                           [0,self.orig_height]])
        self.bottom_x_offset = math.floor((self.orig_width - self.new_width) / 2)
        pts2 = np.float32([[0,0],
                           [self.orig_width, 0],
                           [self.orig_width - self.bottom_x_offset, self.new_height],
                           [self.bottom_x_offset, self.new_height]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(self.img,M,(self.orig_width,self.orig_height))
        self.alt_img = dst
        return dst

    def blobdetect_HSV(self):
        """
    Find blobs (weed) in the altered image (i.e. the foreshortness corrected
    image). Return a cv2 Keypoints list.
    """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(self.alt_img, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        lower_green = np.array([0,100, 100])
        upper_green = np.array([200,255,255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 25

        params.filterByColor = True
        params.blobColor = 255

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        detector=cv2.SimpleBlobDetector_create(params)

        # blur = cv2.GaussianBlur(mask,(10,10),0)
        blur = cv2.medianBlur(mask,1)

        # Detect blobs.
        keypoints = detector.detect(blur)

        return keypoints

    def blobdetect_RGB(self):
        """
    Find blobs (weed) in the altered image (i.e. the foreshortness corrected
    image). Return a cv2 Keypoints list.
    """
        rgb = self.alt_img
        # define range of green color in RGB
        lower_green = np.array([0,130, 0])
        upper_green = np.array([150,255,150])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(rgb, lower_green, upper_green)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 25

        params.filterByColor = True
        params.blobColor = 255

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        detector=cv2.SimpleBlobDetector_create(params)

        # blur = cv2.GaussianBlur(mask,(10,10),0)
        blur = cv2.medianBlur(mask,1)

        # Detect blobs.
        keypoints = detector.detect(blur)

        return keypoints

    
    def robot_img_coords(self):
        Xr_img = math.floor(self.orig_width / 2)
        Yr_img = math.floor(self.new_height / 27.5 * 30)

        return Xr_img, Yr_img

    def movement_vector(self, keypoint):
        Xr_img, Yr_img = self.robot_img_coords()
        Xp_img = keypoint.pt[0]
        Yp_img = keypoint.pt[1]
        Dx_img = Xr_img - Xp_img
        Dy_img = Yr_img - Yp_img
        Dx_wrld = Dx_img * 0.25 / 640  # distance in meters
        Dy_wrld = Dy_img * 0.30 / 480
        angle = math.degrees(math.atan2(Dy_img, Dx_img)) - 90
        distance = math.sqrt((Dx_wrld * Dx_wrld) + (Dy_wrld * Dy_wrld))

        return angle, distance
        

    def archive_image(self, img):
        try:
            imgname = "roboimg" + str(int(time.time())) + ".png"
            imgpath = os.path.join(self.imgdir, imgname)
            # print("Pic name " + imgpath)
            cv2.imwrite(imgpath, img)
        except:
            self.logger.error("archive_image failed %s" % (imgpath))
        
        
