#!/usr/bin/python

import numpy as np
import tracker
import gps3
import utm
import os
import threading
import time

class Kalman:

    def __init__(self):
        self.gps = GPSrunner(1)
        self.gps.start()

    def test(self):
        lat, lon, hdg, vel, tme, mode = self.gps.get_data()
        print(" TESTING: Lat: %s Lon: %s Hdg: %s Vel %s Mode: %s " %
              (lat, lon, hdg, vel, mode))
        print("Time: %s" % (tme))
        
    def reset(self):
        self.gps.stop()
        time.sleep(1)
        self.gps.start()

    def utm_coord(self):
        lat, lon, hdg, vel, tme, mode = self.gps.get_data()
        if isinstance(lat, float) and isinstance(lon, float):
            u = utm.from_latlon(float(lat), float(lon))
            return (u[1], u[0], u[2], u[3]) # North, East, Zone, Zone
        else:
            return (-998.0, -998.0, " ", " ")

    def dump_buf(self, filename):
        i = self.gps.buf_idx
        j = 0
        dump_buf = np.zeros((self.gps.BUFSZ, 5), dtype=float)
        while i < self.gps.BUFSZ:
            for k in range(5):
                dump_buf[j, k] = self.gps.buf[i, k]
            i = i + 1
            j = j + 1
        i = 0
        while i < self.gps.buf_idx:
            for k in range(5):
                dump_buf[j, k] = self.gps.buf[i, k]
            i = i + 1
            j = j + 1
        np.savetxt(filename, dump_buf)            

    def __del__(self):
        self.gps.stop()
        # self.gps.join()

class GPSrunner (threading.Thread):

    def __init__(self, threadID):
        self.BUFSZ = 100
        self.buf = np.zeros((self.BUFSZ, 5), dtype=float)
        self.buf_idx = 0
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.latitude = "no data"
        self.longitude = "no data"
        self.heading = "no data"
        self.speed = "no data"
        self.gpsTime = "no data"
        self.mode = "no data"
        self.quit = False
        self.mutex = threading.Condition()

    def run(self):
        print("GPSrunner starting")
        self.quit = False
        self.gpsd_socket = gps3.GPSDSocket()
        self.gpsd_socket.connect(host='localhost', port=2947)
        self.gpsd_socket.watch()
        self.data_stream = gps3.DataStream()
        try:
            for new_data in self.gpsd_socket:
                if new_data:
                    if self.quit:
                        break;
                    self.data_stream.unpack(new_data)
                    with self.mutex:
                        self.speed = self.data_stream.TPV['speed']
                        self.latitude = self.data_stream.TPV['lat']
                        self.longitude = self.data_stream.TPV['lon']
                        self.altitude = self.data_stream.TPV['alt']
                        self.heading = self.data_stream.TPV['track']
                        self.gpsTime = self.data_stream.TPV['time']
                        self.mode = self.data_stream.TPV['mode']
                    # print("Lat: %s Lon: %s Hdg: %s Time: %s" % (self.latitude, self.longitude, self.heading, self.gpsTime))
                    self.buf[self.buf_idx, 0] = float(time.time())
                    try:
                        self.buf[self.buf_idx, 1] = float(self.latitude)                
                        self.buf[self.buf_idx, 2] = float(self.longitude)
                        self.buf[self.buf_idx, 3] = float(self.heading)
                        # self.buf[self.buf_idx, 4] = self.mode
                    except:
                        self.buf[self.buf_idx, 1] = -998.0
                        self.buf[self.buf_idx, 2] = -998.0
                        self.buf[self.buf_idx, 3] = -998.0
                    self.buf_idx = self.buf_idx + 1
                    if self.buf_idx >= self.BUFSZ:
                        self.buf_idx = 0
                    time.sleep(0.5)
        
        finally:
            self.gpsd_socket.close()
            print('GPSrunner Terminated')

    def get_data(self):
        with self.mutex:
            return (self.latitude, self.longitude, self.heading, self.speed, self.gpsTime, self.mode)
        
    def stop(self):
        self.quit = True
        

