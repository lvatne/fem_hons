#!/usr/bin/python

import matplotlib.path
import numpy as np
import os
import utm

class Geofence:
    """ Maintains the coordinates that make up the fence for the robot
        Loads the geofence coordinates from file:
        $ROBOHOME/conf/geofence.properties
        The coordinates in the file are specified in meters, relative
        to the origo position (i.e. first point is 0.0 , 0.0)
    """

    def __init__(self, N_0, E_0):
        """
        Keyword arguments:
        N_0: the North coordinate of the origo position (the charging station)
        E_0: the East coordinate of the origo position (charging station)
        The N_0 and E_0 are UTM coordinates in meters in the relevant
        UTM zone
        """
        
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
        """
        Keyword arguments:
        x: The UTM East coordinate in meters
        y: The UTM North coordinate in meters
        """
        
        relX = x - self.origoE
        relY = y - self.origoN
        return relX, relY        

    def getfilename(self):
        """ Return the filename of the stored coordinates """
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


