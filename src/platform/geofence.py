#!/usr/bin/python

import matplotlib.path
import numpy as np
import os
import utm

class Geofence:

    def __init__(self):
        self.test_geofence = np.array([[-8.2,28.1], [33.287, 60.44], [45, 40], [88, 40], [88, 125], [55, 125], [31, 97], [-12, 135], [-35, 125], [-70, 75]], float)

        # self.geofence = self.test_geofence
        self.geofence = self.load()
        self.perimeter = matplotlib.path.Path(self.geofence)

    def isinside(self, x, y):
        return self.perimeter.contains_point((x, y))

    def getfilename(self):
        try:
            robohome = os.environ['ROBOHOME']
        except KeyError:
            print("Error in installation. $ROBOHOME does not exist")
            raise
        confdir = os.path.join(robohome, "conf")
        self.fencefilename = os.path.join(confdir, "geofence.properties")
        return self.fencefilename

    def save(self):
        np.savetxt(self.getfilename(), self.geofence)

    def load(self):
        tmp_geofence = np.loadtxt(self.getfilename())
        return tmp_geofence.reshape((tmp_geofence.size // 2), 2)


