import numpy as np
import os
import time
import logging
import cv2
import sysprops

class Camera:

    def __init__(self):
        props = sysprops.SysProps()
        self.cv2_cam_dev1 = props.cv2_cam_device1
        self.imgdir = os.path.join(props.robohome, "img")
        
        self.logger = logging.getLogger('camera_img')        

        try:
            robohome = os.environ['ROBOHOME']
        except KeyError:
            print("Error in installation. $ROBOHOME does not exist (motor_sw)")
            self.logger.error("Error in installation. $ROBOHOME does not exist (motor_sw)")
            raise
        logdir = os.path.join(robohome, "log")
        hdlr = logging.FileHandler(os.path.join(logdir, "camera_img.log"))
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.WARNING)
        self.logger.warning("--------------+++--------------")
        

    def get_picture(self):
        # print("Take pic from device %d" % (self.cv2_cam_dev1))
        try:
            cap = cv2.VideoCapture(self.cv2_cam_dev1)
            ret, frame = cap.read()
            # print("Returned %d" % (ret))
            # imgname = "roboimg" + str(int(time.time())) + ".png"
            # imgpath = os.path.join(self.imgdir, imgname)
            # print("Pic name " + imgpath)
            # cv2.imwrite(imgpath, frame)
            # When everything done, release the capture
        except:
            print("get_picture failed")
            self.logger.error("Get picture failed %d" % (ret))
            raise
        finally:
            cap.release()

        return frame
        # cv2.destroyAllWindows()
        

    def take_picture(self):
        print("Take pic from device %d" % (self.cv2_cam_dev1))
        try:
            cap = cv2.VideoCapture(self.cv2_cam_dev1)
            ret, frame = cap.read()
            print("Returned %d" % (ret))
            imgname = "roboimg" + str(int(time.time())) + ".png"
            imgpath = os.path.join(self.imgdir, imgname)
            print("Pic name " + imgpath)
            cv2.imwrite(imgpath, frame)
            # When everything done, release the capture
        except:
            print("take_picture failed")
            self.logger.error("Take picture failed %s" % (imgpath))
            raise
        finally:
            cap.release()
        # cv2.destroyAllWindows()
