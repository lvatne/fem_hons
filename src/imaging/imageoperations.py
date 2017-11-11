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
        # self.adjust_brightness()

    def adjust_brightness(self):
        img_yuv = cv2.cvtColor(self.img, cv2.COLOR_BGR2YUV)
        # equalize the histogram of the Y channel
        img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
        # convert the YUV image back to RGB format
        self.adj_img = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        self.alt_img = self.adj_img


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
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        # define range of green color in HSV
        # lower_green = np.array([0,100, 100])
        # upper_green = np.array([200,255,255])

        # Empirical value 6/8-2017 :
        lower_green = np.array([9, 74, 170])
        upper_green = np.array([96, 125, 249])
        
        # Threshold the HSV image to get only green colors
        self.mask = cv2.inRange(hsv, lower_green, upper_green)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        params.thresholdStep = 50
        params.minDistBetweenBlobs = 10

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

        # self.blur = cv2.GaussianBlur(mask,(10,10),0)
        # self.blur = cv2.medianBlur(self.mask, 1)
        # self.blur = mask.copy()
        # Detect blobs.
        keypoints = detector.detect(self.mask)

        return keypoints

    def blobdetect_RGB(self):
        """
    Find blobs (weed) in the altered image (i.e. the foreshortness corrected
    image). Return a cv2 Keypoints list.
    """
        rgb = self.alt_img
        # define range of green color in RGB
        lower_green = np.array([0,50, 0])
        upper_green = np.array([150,255,150])

        # Threshold the RGB image to get only green colors
        self.mask = cv2.inRange(rgb, lower_green, upper_green)

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
        self.blur = cv2.medianBlur(self.mask,1)

        # Detect blobs.
        keypoints = detector.detect(self.blur)

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
        """ Save image to disk using the directory specified in the
            properties and a time to generate unique image names
        """
        
        try:
            imgname = "roboimg" + str(int(time.time())) + ".png"
            imgpath = os.path.join(self.imgdir, imgname)
            # print("Pic name " + imgpath)

            cv2.imwrite(imgpath, img)
        except:
            self.logger.error("archive_image failed %s" % (imgpath))

    def nothing(self, x):
        """ Callback function for the trackbars used by the
            gui_choose_hsv() function.
        """
        
        pass

    def gui_choose_hsv(self, img):
        """ Test function for playing with image parameters
            using GUI
        """
        
        cv2.namedWindow('result')

        # Starting with 100's to prevent error while masking
        hl,sl,vl = 100,100,100
        hh,sh,vh = 179, 255, 255

        # Creating track bar
        cv2.createTrackbar('hl', 'result',0,179,self.nothing)
        cv2.createTrackbar('sl', 'result',0,255,self.nothing)
        cv2.createTrackbar('vl', 'result',0,255,self.nothing)
        cv2.createTrackbar('hh', 'result',0,179,self.nothing)
        cv2.createTrackbar('sh', 'result',0,255,self.nothing)
        cv2.createTrackbar('vh', 'result',0,255,self.nothing)

        cv2.setTrackbarPos('hh','result',hh)
        cv2.setTrackbarPos('sh','result',sh)
        cv2.setTrackbarPos('vh','result',vh)


        while(1):

            frame = img.copy()
            #converting to HSV
            hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

            # get info from track bar and appy to result
            hl = cv2.getTrackbarPos('hl','result')
            sl = cv2.getTrackbarPos('sl','result')
            vl = cv2.getTrackbarPos('vl','result')

            hh = cv2.getTrackbarPos('hh','result')
            sh = cv2.getTrackbarPos('sh','result')
            vh = cv2.getTrackbarPos('vh','result')

            # Normal masking algorithm
            lower_green = np.array([hl,sl,vl])
            upper_green = np.array([hh, sh, vh])
            # upper_green = np.array([180,255,255])

            mask = cv2.inRange(hsv,lower_green, upper_green)

            result = cv2.bitwise_and(frame,frame,mask = mask)

            cv2.imshow('result',result)
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
        
