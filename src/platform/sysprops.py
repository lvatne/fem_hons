#!/usr/bin/python


import jprops
import os

class SysProps:

    
    def __init__(self) :
        try:
            self.robohome = os.environ['ROBOHOME']
        except KeyError:
            print("Error in installation. $ROBOHOME does not exist")
            raise
        self.confdir = os.path.join(self.robohome, "conf")
        self.logdir = os.path.join(self.robohome, "log")
        self.imagedir = os.path.join(self.robohome, "img")
        self.sysprops = os.path.join(self.confdir, "robot.properties")
        f = 0
        try:
            f = open(self.sysprops, 'rb')
        except FileNotFoundError:
            self.props = {'home_lat' :  '59.1234', 'home_lon': '5.5678'}

            f = open(self.sysprops, 'wb')
            jprops.store_properties(f, self.props)
            f.close()
            f = open(self.sysprops, 'rb')

        self.props = jprops.load_properties(f)
        f.close()
        self.refresh()
        self.conf = self.confdir

    def store(self):
        f=open(self.sysprops, 'wb')
        jprops.store_properties(f, self.props)
        f.close()

    def refresh(self):
        #
        # Instantiate all persistent properties here:
        #
        self.home_lat = float(self.props['home_lat'])
        self.home_lon = float(self.props['home_lon'])
        self.x_offset = float(self.props['x_offset'])
        self.y_offset = float(self.props['y_offset'])
        self.xy_ratio = float(self.props['xy_ratio'])
        self.motor_min_power = float(self.props['motor_min_power'])
        self.cam_device1=self.props['cam_device1']
        self.cam_width1=int(self.props['cam_width1'])
        self.cam_height1=int(self.props['cam_height1'])
        self.cv2_cam_device1=int(self.props['cv2_cam_device1'])
        self.left_forward=int(self.props['left_forward'])
        self.left_reverse=int(self.props['left_reverse'])
        self.right_forward=int(self.props['right_forward'])
        self.right_reverse=int(self.props['right_reverse'])
        

    def set(self, key, value):
        self.props[key] = str(value)
        self.refresh()
        
    def test(self):
        self.props = {'home_lat' :  '59.1234', 'home_lon': '5.5678'}
        f=open(self.sysprops, 'wb')
        jprops.store_properties(f, self.props)
        f.close()

    
