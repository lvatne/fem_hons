#!/usr/bin/python

import matplotlib.path
import numpy as np
import os
import utm

class Geofence:

    def __init__(self, N_0, E_0):
        self.test_geofence = np.array([[1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [-1.0, 1.0]], float)
        self.origoN = N_0
        self.origoE = E_0
        # self.geofence = self.test_geofence
        self.geofence = self.load()
        self.perimeter = matplotlib.path.Path(self.geofence, closed=True)

    def isinside(self, x, y):
        """ Takes UTM North and East coordinates. Recalculates to
            relative coordinates """
        relX = x - self.origoE
        relY = y - self.origoN
        return self.perimeter.contains_point((relX, relY))

    def getRelXY(self, x, y):
        relX = x - self.origoE
        relY = y - self.origoN
        return relX, relY        

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
        """ Loads a geofence in relative coordinates. Order is X1 Y1 X2 Y2 etc
            where X coordinates are East, Y coordinates are North """
        tmp_geofence = np.loadtxt(self.getfilename())
        return tmp_geofence.reshape((tmp_geofence.size // 2), 2)


