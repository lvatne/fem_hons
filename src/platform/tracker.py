#!/usr/bin/python

import numpy as np
import smbus
import math
import logging
import struct
import time
import sys
import os
import logging
import motor_sw
import geofence
import sysprops

class Tracker:
    """ Tracker class: Handles communication with on-board inertial
    sensors and implements the PID regulator that maintains a straight course.
    """
    def __init__(self):
        self.BUFSZ = 100
        self.PV_buf = np.zeros(self.BUFSZ, dtype=float) # PV values for the PID regulator
        self.P_buf = np.zeros(self.BUFSZ, dtype=float) # P values for PID regulator
        self.I_buf = np.zeros(self.BUFSZ, dtype=float) # I values for the PID regulator
        self.D_buf = np.zeros(self.BUFSZ, dtype=float) # D values for the PID regulator
        self.E_buf = np.zeros(self.BUFSZ, dtype=float) # Error values for the PID regulator
        self.CO_buf = np.zeros(self.BUFSZ, dtype=float) # Controller Output from the PID regulator
        self.gyro_buf = np.zeros(self.BUFSZ, dtype=float) # raw gyro values
        self.acc_buf = np.zeros(self.BUFSZ, dtype=float) # raw accleleration values
        self.tstamp = np.zeros(self.BUFSZ, dtype=float) # Time stamps for samples
        self.Kp = 5
        self.Ki = 2
        self.Kd = 1
        
        self.buf_idx = 0 # We're at the same place in all buffers
        self.Fs = 50.0 # sample rate, Hz
        self.Ts = 1.0 / self.Fs # Pre calculate sleep time
        self.Ta = self.Ts # Will be filled in with actual sampling period

        self.dbg_cnt = 0
        
        self.cur_x = 70.0
        self.cur_y = 70.0
        self.distance = 0.0

        self.cur_speed = 3.0
        self.cur_hdg = 0
        self.c = Compass()
        self.a = Accelerometer()
        self.g = Gyro()
        self.m = motor_sw.Motor_sw()
        # Get calibrated acceleration offset
        self.x_offset = 0.0
        for i in range(5):
            acc = self.a.getAxes()
            self.x_offset = self.x_offset + acc[0]
            time.sleep(0.1)
        self.x_offset = self.x_offset / 5            
        self.logger = logging.getLogger('navigation')
        self.logger.info("--------------+++--------------")
        
    def stop(self):
        self.m.signal(self.m.STOP, 0)

    def move_dist(self, meter):
        """
        Returns 0 on success. Will make several attempts at honouring the request.
        In case the robot is stuck or similar, various methods will be employed to
        free the wheels or whatever
        """
        recover_attempts = 0
        retval = self.move_dist_basic(meter)
        while retval > 0 and recover_attempts < 5:
            recover_attempts = recover_attempts + 1
            if recover_attempts == 1:
                self.logger.warning("move_dist(%f) recovery #1 (reverse)" % (meter))
                self.m.signal(self.m.STOP, 0)
                self.m.signal(self.m.RUN_REV, 85)
                time.sleep(0.1)
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.1)
                retval = self.move_dist_basic(meter)
            elif recover_attempts == 2:
                self.logger.warning("move_dist(%f) recovery #2 (turn right)" % (meter))
                self.m.signal(self.m.STOP, 0)
                self.m.signal(self.m.TURN_RIGHT, 85)
                time.sleep(0.1)
                self.m.signal(self.m.TURN_LEFT, 85)
                time.sleep(0.1)                
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.1)
                retval = self.move_dist_basic(meter)
            elif recover_attempts == 3:
                self.logger.warning("move_dist(%f) recovery #3 (turn left)" % (meter))
                self.m.signal(self.m.STOP, 0)
                self.m.signal(self.m.TURN_LEFT, 85)
                time.sleep(0.1)
                self.m.signal(self.m.TURN_RIGHT, 85)
                time.sleep(0.1)                
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.1)
                retval = self.move_dist_basic(meter)
            elif recover_attempts == 4:
                self.logger.warning("move_dist(%f) recovery #4 (reverse more)" % (meter))
                self.m.signal(self.m.STOP, 0)
                self.m.signal(self.m.RUN_REV, 100)
                time.sleep(0.3)
                self.m.signal(self.m.RUN_FWD, 100)
                time.sleep(0.3)
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.1)
                retval = self.move_dist_basic(meter)
            elif recover_attempts >= 5:
                self.logger.error("move_dist(%f) failed after 5 attempts" % (meter))
        return retval

    def move_dist_basic(self, meter):
        self.dbg_cnt = 0
        self.buf_reset()
        self.buf_idx = 1
        old_dist = self.distance
        r_dist = meter
        off_course_angle = 0
        self.Ta = self.Ts # Will be filled in with actual sampling period
        num_iter = 0
        failure = 0
        self.logger.info("Tracker.move_dist(%3.3f)" % (meter))
        try:
            num_iter = 0 # Count the number of attempts
            # Assume we're stationary, get the offset value            
            acc = self.a.getAxes()
            x_offset = acc[0] # Set instantaneous offset at start
            adj_accel_data = 0
            adj_gyro_data = 0
            vel = 0
            setpoint = 0 # Set point for the PID regulator
            prev_vel = 0
            while math.fabs(r_dist) > 0.05 and num_iter < 2000:
                if r_dist > 0.05:
                    power = self.m.MIN_PWR + r_dist * 200
                    if power > 100:
                        power = 100
                elif r_dist < -0.05:
                    power = 0.0 - self.m.MIN_PWR + r_dist * 200
                    if power < -100:
                        power = -100
                prev_vel = vel
                if power >= 0:
                    self.m.signal(self.m.RUN_FWD, power)
                else:
                    self.m.signal(self.m.RUN_REV, power)
                adj_accel_data, adj_gyro_data = self.keep_running(x_offset, setpoint)
                vel = vel + adj_accel_data * self.Ta
                r_dist = self.remaining_dist(vel, prev_vel, r_dist)
                PID_val = self.PID(setpoint)
                # print("m_fwd: vel: %3.3f, r_dist: %3.3f, PID: %3.3f" % (vel, r_dist, PID_val))
                if PID_val >= 0:
                    if PID_val > 100:
                        PID_val = 100
                    self.m.signal(self.m.STEER_RIGHT, PID_val)
                else:
                    if PID_val < -100:
                        PID_val = -100
                    self.m.signal(self.m.STEER_LEFT, 0.0 - PID_val)

                num_iter = num_iter + 1
                if self.dbg_cnt % 50 == 0:
                    self.logger.info("m_fwd: vel: %3.3f, r_dist: %3.3f, PID: %3.3f" % (vel, r_dist, PID_val))
            self.m.signal(self.m.STOP, 0)
            self.logger.info("Tracker.move_dist(%3.3f) done: vel: %3.3f, r_dist: %3.3f, PID: %3.3f" % (meter, vel, r_dist, PID_val))

        except:
            self.m.signal(self.m.STOP, 0)
            self.logger.error("Tracker.move_dist(%3.3f) failed" % (meter))
            print("ERROR: Tracker.move_dist(%3.3f) failed" % (meter))
            failure = 1
        if num_iter >= 2000 or failure == 1:
            return 1
        else:
            return 0

    def PID(self, setpoint):
        """ Calculate the error value to apply to the steering using P_ I_ and D_buf class buffers.
        The buffer index has already been incremented, so go back one.
        Return a Controler Output (CO) in the range -100 to +100
        """
        errval = 0.0
        idx = self.buf_idx - 1
        if self.buf_idx > 1:
            errval = self.E_buf[idx] * self.Kp
            # Sum up the I values
            Ival = 0.0
            if idx > 10:
                for i in range(10):
                    Ival = Ival + self.I_buf[idx - i]
            else:
                for i in range(idx):
                    Ival = Ival + self.I_buf[i]
            errval = errval + Ival * self.Ki
            errval = errval + self.D_buf[idx] * self.Kd
            # if self.dbg_cnt % 25 == 0:
            # print("Gyro: %3.1f Steer: %3.1f PIDraw: %3.1f %3.1f %3.1f  PIDgain: %3.1f %3.1f %3.1f tstamp: %f" %
            #      (self.gyro_buf[idx], setpoint - errval, self.E_buf[idx], Ival, self.D_buf[idx], self.E_buf[idx] * self.Kp, Ival * self.Ki, self.D_buf[idx] *self.Kd, time.time()))
                
        CO = setpoint - errval
        if CO < -100:
            CO = -100.0
        elif CO > 100:
            CO = 100.0
        return CO
            
    def keep_running(self, x_offset, setpoint):
        """ Sample data and prepare the values for the PID regulator.
        Acceleration data and gyro data is sampled at Fs
        Operates directly on the class buffers: acc_buf, gyro_buf, P_buf, I_buf and D_buf
        """
        
        if self.Ta < self.Ts:
            time.sleep(self.Ts - self.Ta) # maintain sampling rate
        idx = self.buf_idx    
        acc = self.a.getAxes()
        self.acc_buf[idx] = acc[0] - self.x_offset # Apply calibration
        gyr = self.g.read()
        self.tstamp[idx] = time.time()
        instant_gyro = gyr[2] # It's the Z rotation we're using
        if idx > 0:
            self.Ta = self.tstamp[idx] - self.tstamp[idx - 1]
            if self.Ta < 0.0:
                self.Ta = math.fabs(self.Ta)
            self.gyro_buf[idx] = self.gyro_buf[idx - 1] + instant_gyro * self.Ta
        else:
            # self.Ta = self.Ts
            self.gyro_buf[idx] = instant_gyro * self.Ta
        # filter the data
        if idx > 4:
            # Hanning filter (moving average window of 5 with 2x weight on current value)
            adj_accel = (self.acc_buf[idx-4] + self.acc_buf[idx-3] + self.acc_buf[idx-2] +
                         self.acc_buf[idx-1] + 2 * self.acc_buf[idx]) / 6
            # adj_gyro = (gyro_buf[idx-4] + gyro_buf[idx-3] + gyro_buf[idx-2] + gyro_buf[idx-1] + 2 * gyro_buf[idx]) / 6
            adj_gyro = self.gyro_buf[idx] # Just using raw gyro value without filtering. Internal filter in chip
        else:
            adj_accel = self.acc_buf[idx]
            adj_gyro = self.gyro_buf[idx]

        # Pre calculate values for the PID regulator
        self.PV_buf[idx] = adj_gyro
        if idx > 0:
            self.E_buf[idx] = setpoint - adj_gyro # i.e. self.PV_buf[idx]
            self.I_buf[idx] = (self.E_buf[idx] - self.E_buf[idx-1]) * (self.Ta / 2) + min(self.E_buf[idx], self.E_buf[idx-1]) * self.Ta
            self.D_buf[idx] = (self.E_buf[idx]- self.E_buf[idx-1]) / self.Ta
        else:
            self.I_buf[idx] = 0.0
            self.D_buf[idx] = 0.0
            
        if self.buf_idx < self.BUFSZ - 2:
            self.buf_idx = self.buf_idx + 1
        else:  # Move the last of the buffer down to the first 3 cells and wrap the index
            self.buf_idx = 2
            for i in range(3):
                tgt_i = self.BUFSZ - 5 + i
                self.E_buf[i] = self.E_buf[tgt_i]
                self.I_buf[i] = self.I_buf[tgt_i]
                self.D_buf[i] = self.D_buf[tgt_i]
                self.tstamp[i] = self.tstamp[tgt_i]

        # if self.dbg_cnt == 99:
        #     np.savetxt("gyro_buf.txt", self.gyro_buf)
        #     np.savetxt("PV_buf.txt", self.PV_buf)
        #     np.savetxt("E_buf.txt", self.E_buf)
        #     np.savetxt("I_buf.txt", self.I_buf)
        #     np.savetxt("D_buf.txt", self.D_buf)
        if self.dbg_cnt % 50 == 0:
            self.logger.info("Ta = %3.5f Fa = %3.3f Hz" % (self.Ta, 1.0 / self.Ta))
        self.dbg_cnt = self.dbg_cnt + 1
        return adj_accel, adj_gyro

    def remaining_dist(self, velocity, prev_velocity, prev_distance):
        remaining = prev_distance - self.Ta * (prev_velocity + 0.5 * (velocity - prev_velocity))
        return remaining

    def set_heading(self, hdg):
        if hdg > 180:
            tmp = hdg - 180
            hdg = -180 + tmp
        if hdg < -180:
            tmp = hdg + 180
            hdg = 180 - tmp
        old_hdg = self.c.local_heading_deg()
        turn_angle = self.diffheading(hdg, old_hdg)
        return self.turn(turn_angle, hdg)

    def diffheading(self, hdg, old_hdg):
        turn_angle = hdg - old_hdg
        if turn_angle > 180:
            turn_angle = -1 *(360 - turn_angle)
        elif turn_angle < -180:
            turn_angle = 360 - math.fabs(turn_angle)
        return turn_angle        

    def turn_relative(self, rel_angle):
        """ Turn the number of degrees specified, between -180 and 180 while standing still
            Make several attempts, in case the wheels are stuck or slipping etc. Return the
            remaining number of degrees
        """
        recover_attempts = 0
        remaining_angle = self.turn_relative_basic(rel_angle)
        while math.fabs(remaining_angle) > 2 and recover_attempts < 5:
            recover_attempts = recover_attempts + 1
            if recover_attempts == 1:
                self.logger.warning("turn_relative(%f) recovery #1 (reverse)" % (rel_angle))
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                self.m.signal(self.m.RUN_REV, 85)
                time.sleep(0.2)
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.2)
                remaining_angle = self.turn_relative_basic(remaining_angle)
            elif recover_attempts == 2:
                self.logger.warning("turn_relative(%f) recovery #2 (turn right)" % (rel_angle))
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                self.m.signal(self.m.TURN_RIGHT, 85)
                time.sleep(0.2)
                self.m.signal(self.m.TURN_LEFT, 85)
                time.sleep(0.2)                
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                remaining_angle = self.turn_relative_basic(remaining_angle)
            elif recover_attempts == 3:
                self.logger.warning("turn_relative(%f) recovery #3 (turn left)" % (rel_angle))
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                self.m.signal(self.m.TURN_LEFT, 85)
                time.sleep(0.2)
                self.m.signal(self.m.TURN_RIGHT, 85)
                time.sleep(0.2)                
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                remaining_angle = self.turn_relative_basic(remaining_angle)
            elif recover_attempts == 4:
                self.logger.warning("turn_relative(%f) recovery #4 (reverse more)" % (rel_angle))
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                self.m.signal(self.m.RUN_REV, 100)
                time.sleep(0.3)
                self.m.signal(self.m.RUN_FWD, 100)
                time.sleep(0.3)
                self.m.signal(self.m.STOP, 0)
                time.sleep(0.5)
                remaining_angle = self.turn_relative_basic(remaining_angle)
            elif recover_attempts >= 5:
                self.logger.error("turn_relative(%f) failed after 5 attempts" % (rel_angle))               
        return remaining_angle
            
        


    def turn_relative_basic(self, rel_angle):
        """ Turn the number of degrees specified, between -180 and 180 while standing still
            The latest value of gyro_buf gives the number of degrees turned so far.
            Adjust power to the wheels relative to how many degrees remaining to turn
            Return the remaining angle
        """
        giveup = 500
        idx = 1
        self.buf_reset()
        self.tstamp[0] = time.time()
        self.gyro_buf[0] = 0.0
        rel_angle = rel_angle % 360
        if rel_angle > 180:
            rel_angle = rel_angle - 360
        elif rel_angle < -180:
            rel_angle = rel_angle + 360
        turn_angle = rel_angle # turn_angle is the remaining number of degrees
        if turn_angle > 2:
            self.m.signal(self.m.TURN_RIGHT, 100)
        elif turn_angle < -2:
            self.m.signal(self.m.TURN_LEFT, 100)
        while turn_angle > 0.5 or turn_angle < -0.5:
            gyr = self.g.read()
            self.tstamp[idx] = time.time()
            instant_gyro = gyr[2] # It's the Z rotation we're using. In deg/sec
            self.Ta = self.tstamp[idx] - self.tstamp[idx - 1]
            if self.Ta < 0.0:
                self.Ta = math.fabs(self.Ta)
            self.gyro_buf[idx] = self.gyro_buf[idx - 1] + instant_gyro * self.Ta
            turn_angle = rel_angle + self.gyro_buf[idx]
            power = math.fabs(turn_angle / 0.8)
            if power < self.m.MIN_PWR:
                power = self.m.MIN_PWR
            elif power > 100:
                power = 100
            # print("turn_angle: %3.3f turned so far %3.3f, instant %3.3f, power: %3.3f" % (turn_angle, self.gyro_buf[idx], instant_gyro, power))
            if turn_angle > 2:
                self.m.signal(self.m.TURN_RIGHT, power)
            elif turn_angle < -2:
                self.m.signal(self.m.TURN_LEFT, power)
            idx = idx + 1
            if idx > self.BUFSZ - 2: # Wrap the buffers around
                self.tstamp[0] = self.tstamp[idx - 1]
                self.gyro_buf[0] = self.gyro_buf[idx - 1]
                idx = 1
            if self.Ta < self.Ts:
                time.sleep(self.Ts - self.Ta)
            giveup = giveup - 1
            if giveup < 0:
                print("tracker.turn_relative(%3.3f) Giving up with angle %3.3f" % (rel_angle, turn_angle))
                self.m.force_stop()
                break
        print("turn_angle: %3.3f turned so far %3.3f, instant %3.3f, power: %3.3f" % (turn_angle, self.gyro_buf[idx-1], instant_gyro, power))
        self.m.stop()
        return turn_angle
                
        
    def turn(self, angle, hdg):
        """ Adjust the relative power of the motors while running forward
        """
        for i in range(20):
            adj_accel_data, adj_gyro_data = self.keep_running(0, angle)
            self.m.steer(self.PID(angle))

    def buf_reset(self):
        for i in range(5):
            self.PV_buf[i] = 0.0  # PV values for the PID regulator
            self.P_buf[i] = 0.0 # P values for PID regulator
            self.I_buf[i] = 0.0 # I values for the PID regulator
            self.D_buf[i] = 0.0 # D values for the PID regulator
            self.E_buf[i] = 0.0 # Error values for the PID regulator
            self.CO_buf[i] = 0.0 # Controller Output from the PID regulator
            self.gyro_buf[i] = 0.0 # raw gyro values
            self.acc_buf[i] = 0.0 # raw accleleration values
            self.tstamp[i] = time.time() # Time stamps for samples

    def dump_buf(self, filename):
        i = self.buf_idx
        j = 0
        dump_buf = np.zeros((self.BUFSZ, 3), dtype=float)
        while i < self.BUFSZ:
            dump_buf[j, 0] = self.tstamp[i]
            dump_buf[j, 1] = self.gyro_buf[i]
            dump_buf[j, 2] = self.acc_buf[i]
            j = j + 1
            i = i + 1
        i = 0
        while i < self.buf_idx:
            dump_buf[j, 0] = self.tstamp[i]
            dump_buf[j, 1] = self.gyro_buf[i]
            dump_buf[j, 2] = self.acc_buf[i]
            j = j + 1
            i = i + 1
        np.savetxt(filename, dump_buf)
            
        

class Gyro:
    """ Interface to the ITG3205 gyro chip on the 9-degrees of freedom
    Subassembly (GY-85).

    
    Positive Z values: Counter-clockwise rotation
    Negative Z values: clockwise rotation
    """
    
    def __init__(self, address = 0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        #self.scalar = 14.375
        self.scalar = 12.12  # Determined empirically
        # Select Power management register 0x3E(62)
        #		0x01(01)	Power up, PLL with X-Gyro reference
        self.bus.write_byte_data(self.address, 0x3E, 0x01)
        # Select DLPF register, 0x16(22)
        #		0x18(24)	Gyro FSR of +/- 2000 dps
        self.bus.write_byte_data(self.address, 0x16, 0x18)

    def self_test(self):
        value = self.bus.read_byte_data(self.address, 0)
        print("Gyro self test %4X" % (value))
        if value == 0x68:
            return True
        return False

    def read(self):
        # Read data back from 0x1D(29), 6 bytes
        # X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
        data = self.bus.read_i2c_block_data(self.address, 0x1D, 6)

        # Convert the data
        xGyro = data[0] * 256 + data[1]
        if xGyro > 32767 :
            xGyro -= 65536
        xGyro = xGyro / self.scalar # To get degrees per second
        yGyro = data[2] * 256 + data[3]
        if yGyro > 32767 :
            yGyro -= 65536
        yGyro = yGyro / self.scalar
        zGyro = data[4] * 256 + data[5]
        if zGyro > 32767 :
            zGyro -= 65536
        zGyro = zGyro / self.scalar
        #return (xGyro, yGyro, zGyro)
        return np.array([xGyro, yGyro, zGyro])

    def read_temp(self):
        data = self.bus.read_i2c_block_data(self.address, 0x1B, 2)
        temp = data[0] * 256 + data[1]
        if temp > 32767 :
            temp -= 65536
        return temp



class Accelerometer:
    EARTH_GRAVITY_MS2   = 9.80665
    SCALE_MULTIPLIER    = 0.004

    DATA_FORMAT         = 0x31
    BW_RATE             = 0x2C
    POWER_CTL           = 0x2D

    BW_RATE_1600HZ      = 0x0F
    BW_RATE_800HZ       = 0x0E
    BW_RATE_400HZ       = 0x0D
    BW_RATE_200HZ       = 0x0C
    BW_RATE_100HZ       = 0x0B
    BW_RATE_50HZ        = 0x0A
    BW_RATE_25HZ        = 0x09

    RANGE_2G            = 0x00
    RANGE_4G            = 0x01
    RANGE_8G            = 0x02
    RANGE_16G           = 0x03

    MEASURE             = 0x08
    AXES_DATA           = 0x32

    address = None

    def __init__(self, address = 0x53):
        self.bus = smbus.SMBus(1)

        self.address = address
        self.setBandwidthRate(self.BW_RATE_100HZ)
        self.setRange(self.RANGE_2G)
        self.enableMeasurement()

    def enableMeasurement(self):
        self.bus.write_byte_data(self.address, self.POWER_CTL, self.MEASURE)

    def setBandwidthRate(self, rate_flag):
        self.bus.write_byte_data(self.address, self.BW_RATE, rate_flag)

    # set the measurement range for 10-bit readings
    def setRange(self, range_flag):
        # value = self.bus.read_byte_data(self.address, self.DATA_FORMAT)
        # value &= ~0x0F;
        # value |= range_flag;  
        # value |= 0x08;
        self.bus.write_byte_data(self.address, self.DATA_FORMAT, range_flag)
    
    # returns the current reading from the sensor for each axis
    #
    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs
    def getAxes(self, gforce = False):
        ubytes = self.bus.read_i2c_block_data(self.address, self.AXES_DATA, 6)
        
        x = ubytes[0] | (ubytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = ubytes[2] | (ubytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = ubytes[4] | (ubytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)

        x = x * self.SCALE_MULTIPLIER 
        y = y * self.SCALE_MULTIPLIER
        z = z * self.SCALE_MULTIPLIER

        if gforce == False:
            x = x * self.EARTH_GRAVITY_MS2
            y = y * self.EARTH_GRAVITY_MS2
            z = z * self.EARTH_GRAVITY_MS2

        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)
        #print("scaled: %4f %4f %4f" % (x, y, z))
        return np.array([x, y, z])
        # return {x, y, z}

    def read_raw(self):
        ubytes = self.bus.read_i2c_block_data(self.address, self.AXES_DATA, 6)
        # print("X: %3X %3X Y: %3X %3X Z: %3X %3X" % (ubytes[1], ubytes[0], ubytes[3], ubytes[2], ubytes[5], ubytes[4]))
        x = ubytes[0] | (ubytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = ubytes[2] | (ubytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = ubytes[4] | (ubytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)
            
        # print("X: %X Y: %X Z: %X" %  (x, y, z))
        x = x * self.SCALE_MULTIPLIER * self.EARTH_GRAVITY_MS2
        y = y * self.SCALE_MULTIPLIER * self.EARTH_GRAVITY_MS2
        z = z * self.SCALE_MULTIPLIER * self.EARTH_GRAVITY_MS2
        # print("scaled: %4f %4f %4f" % (x, y, z))


class Compass:

    # vim: set fileencoding=UTF-8 :

    # HMC5888L Magnetometer (Digital Compass) wrapper class
    # Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
    # but uses smbus rather than quick2wire and sets some different init
    # params.

    # originally class hmc5883l:

    # Norway: gauss = 0.507, declination = 0.88 deg

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0)):
        self.bus = smbus.SMBus(port)
        self.address = address
        
        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement
        s = sysprops.SysProps()
        self.x_offset = s.x_offset
        self.y_offset = s.y_offset
        self.xy_ratio = s.xy_ratio
        self.logger = logging.getLogger('navigation')
        try:
            robohome = os.environ['ROBOHOME']
        except KeyError:
            print("Error in installation. $ROBOHOME does not exist (motor_sw)")
            self.logger.error("Error in installation. $ROBOHOME does not exist (motor_sw)")
            raise
        logdir = os.path.join(robohome, "log")

        hdlr = logging.FileHandler(os.path.join(logdir, "navigation.log"))
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.INFO)
        self.self_test()


    def local_heading_deg(self):
        h = self.degrees(self.heading())
        hdg = h[0]
        # hdg += 90
        # if hdg > 180:
        #    tmp = hdg - 180
        #    hdg = -180 + tmp
        # elif hdg < -180:
        #    tmp = hdg + 180
        #    hdg = 180 - tmp
        return hdg            

    def self_test(self):
        # self.bus.write_byte_data(self.address, 0x00, 0x70)
        # self.bus.write_byte_data(self.address, 0x01, 0x00)
        # self.bus.write_byte_data(self.address, 0x02, 0x00)


        id_a = self.bus.read_byte_data(self.address, 0x0A)  # should return 48H
        id_b = self.bus.read_byte_data(self.address, 0x0B)  # should return 34H
        id_c = self.bus.read_byte_data(self.address, 0x0C)  # should return 33H

        if id_a == 0x48 and id_b == 0x34 and id_c == 0x33:
            print("Compass self test OK: A=%2XH B=%2XH C=%2xH " % (id_a, id_b, id_c))
            self.logger.info("Compass self test OK: A=%2XH B=%2XH C=%2xH " % (id_a, id_b, id_c))
            return True
        else:
            print("Compass self test failed: A=%2XH B=%2XH C=%2xH Should have been A=48H B=34H C=33H" % (id_a, id_b, id_c))
            self.logger.info("Compass self test failed: A=%2XH B=%2XH C=%2xH Should have been A=48H B=34H C=33H" % (id_a, id_b, id_c))
            return False

    def reset(self):
        self.bus.write_byte_data(self.address, 0x02, 0x03) # Set in idle mode
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def test_read(self):
        x_axis_msb = self.bus.read_byte_data(self.address, 0x03)
        x_axis_lsb = self.bus.read_byte_data(self.address, 0x04)

        z_axis_msb = self.bus.read_byte_data(self.address, 0x05)
        z_axis_lsb = self.bus.read_byte_data(self.address, 0x06)

        y_axis_msb = self.bus.read_byte_data(self.address, 0x07)
        y_axis_lsb = self.bus.read_byte_data(self.address, 0x08)

        status = self.bus.read_byte_data(self.address, 0x09)

        # x_axis = np.int32((x_axis_msb * 256) + x_axis_lsb)
        # y_axis = np.int32((y_axis_msb * 256) + y_axis_lsb)
        # z_axis = np.int32((z_axis_msb * 256) + z_axis_lsb)

        x_axis = self.twos_complement(x_axis_msb << 8 | x_axis_lsb, 16)
        y_axis = self.twos_complement(y_axis_msb << 8 | y_axis_lsb, 16)
        z_axis = self.twos_complement(z_axis_msb << 8 | z_axis_lsb, 16)

        raw_heading = math.atan2(y_axis * self.__scale, x_axis * self.__scale)

        print("X MSB %4X X LSB %4X" % (x_axis_msb, x_axis_lsb))
        print("Y MSB %4X Y LSB %4X" % (y_axis_msb, y_axis_lsb))
        print("Z MSB %4X Z LSB %4X" % (z_axis_msb, z_axis_lsb))
        print("X %4d Y %4d Z %4d" % (x_axis, y_axis, z_axis))
        print("Raw heading %2.4f rad %3.2d deg" % (raw_heading, raw_heading * (180.0 / math.pi)))

        print("Status %4X" % (status))
        
        
    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        #print map(hex, data)
        x = (self.__convert(data, 3) - self.x_offset)
        y = (self.__convert(data, 7) - self.y_offset) * self.xy_ratio
        z = self.__convert(data, 5) * self.__scale
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x)
        headingRad += self.__declination

        # Correct for reversed heading
        # if headingRad < 0:
        #     headingRad += 2 * math.pi

        # Check for wrap and compensate
        # elif headingRad > 2 * math.pi:
        #     headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "Declination: " + self.degrees(self.declination()) + "\n" \
               "Heading: " + self.degrees(self.heading()) + "\n"
