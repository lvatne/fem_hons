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
import ICM20948
import GY85

class Tracker:
    """ Tracker class: Handles communication with on-board inertial
    sensors and implements the PID regulators for maintaining a straight course,
    roll the desired distance and rotate the desired number of degrees.
    """
    def __init__(self):
        self.BUFSZ = 5
        # Accessors for the inertial_data cells. Refers to the index of the inertial_data array
        self.tstamp  = 0
        self.acc_x   = 1
        self.acc_y   = 2
        self.acc_z   = 3
        self.gyr_x   = 4
        self.gyr_y   = 5
        self.gyr_z   = 6
        self.angle_x = 7   # Number of degrees turned since previous sample
        self.angle_y = 8
        self.angle_z = 9
        self.vel_x   = 10
        self.vel_y   = 11
        self.vel_z   = 12
        self.steer_P = 13
        self.steer_I = 14
        self.steer_D = 15
        self.dist_P  = 16
        self.dist_I  = 17
        self.dist_D  = 18
        self.rot_P   = 19
        self.rot_I   = 20
        self.rot_D   = 21
        self.samplingtime=22
        self.tsleep = 23
        self.dist_x = 24
        self.dist_y = 25
        self.dist_z = 26
        self.flt_acc_x = 27
        self.flt_acc_y = 28
        self.flt_acc_z = 29
        self.endmarker = 30


        self.inertial_data = np.zeros((self.BUFSZ, self.endmarker), dtype=float)
        
        self.first_run = True
        self.Fs = 200.0 # sample rate, Hz
        self.Ts = 1.0 / self.Fs # Pre calculate sampling period
        self.Ta = 0.0 # Will be filled in with actual sampling period
        self.sensor_read_time = 0.003 # Should maybe be measured? Or taken from sysprops?
        
        self.Kp_steer = 5
        self.Ki_steer = 2
        self.Kd_steer = 1

        self.Kp_turn = 3
        self.Ki_turn = 2
        self.Kd_turn = 1

        self.Kp_dist = 300
        self.Ki_dist = 200
        self.Kd_dist = 200

        self.s = sysprops.SysProps()
        self.collect_PID_data = self.s.collect_PID_data
        if self.collect_PID_data:
            self.PID_store = np.empty((2000, self.endmarker), dtype=float)
        
        self.dbg_cnt = 0
        self.alpha = 0.05  # Damping factor for the accelerometer low-pass filter
        #
        self.cur_x = 70.0
        self.cur_y = 70.0
        self.distance = 0.0

        self.cur_speed = 3.0
        self.cur_hdg = 0
        self.c = None  # Compass
        self.a = None  # Accelerator
        self.g = None  # Gyro
        self.p = None  # Pressure sensor
        if self.s.inertial_navigation == 'GY85':
            self.c = GY85.Compass_GY85()
            self.a = GY85.Accelerometer_GY85()
            self.g = GY85.Gyro_GY85()
        elif self.s.inertial_navigation == 'ICM20948':
            self.c = ICM20948.Compass_ICM20948()
            self.a = ICM20948.Accelerometer_ICM20948()
            self.g = ICM20948.Gyro_ICM20948()
            self.p = ICM20948.Pressure_ICM20948()
        else:
            self.c = Compass()
            self.a = Accelerometer()
            self.g = Gyro()
            
        self.m = motor_sw.Motor_sw()
        self.calibrate_accelerometer_offset()
        
        self.logger = logging.getLogger('navigation')
        self.logger.info("--------------+++--------------")
        
    def stop(self):
        self.m.signal(self.m.STOP, 0)

    def calibrate_accelerometer_offset(self) :
        revs = 10
        # Get calibrated acceleration offset
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        for i in range(revs):
            time.sleep(0.03)
            acc = self.a.getAxes()
            self.x_offset = self.x_offset + acc[0]
            self.y_offset = self.y_offset + acc[1]
            self.z_offset = self.z_offset + acc[2]
        self.x_offset = self.x_offset / revs
        self.y_offset = self.y_offset / revs
        self.z_offset = self.z_offset / revs
        

    def move_dist(self, meter):
        """
        Returns 0 on success. Will make several attempts at honouring the request.
        In case the robot is stuck or similar, various methods will be employed to
        free the wheels or whatever
        """
        self.inertial_data[0, self.steer_P] = 0.0
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
        """
        Move the specified distance in meters. Forward or reverse.
        Employs two PID controllers; one to go the specified distance, and one
        to run in a straight line (PIDsteer)
        """
        self.dbg_cnt = 0
        self.prev_power = 0.0
        self.buf_reset()
        r_dist = meter
        off_course_angle = 0
        self.Ta = 0.0 # Will be filled in with actual sampling period
        self.first_run = True
        num_iter = self.calc_bailout_dist(meter)
        failure = False
        arrived = False
        self.logger.info("Tracker.move_dist(%3.3f), %d iterations" % (meter, num_iter))
        self.t_now = time.time()
        self.calibrate_accelerometer_offset()
        try:
            velocity = 0.0
            steer_setpoint = 0.0 # Set point for the steering PID regulator
            while not arrived:
                power = self.PIDdist(r_dist)
                if power > 100:
                    power = 100
                if power < -100:
                    power = -100
                self.get_inertial_measurements(self.m) # Initialise inertial_buffer and 
                self.get_inertial_measurements(self.m) # get the sampling started
                if math.floor(power) != math.floor(self.prev_power): # Don't change the power setting unneccessarily
                    if power > 0:
                        self.m.signal(self.m.RUN_FWD, power)
                    else:
                        self.m.signal(self.m.RUN_REV, 0.0 - power)
                self.prev_power = power
                
                self.get_inertial_measurements(self.m)
                
                r_dist = meter - self.inertial_data[0, self.dist_x]
                
                PID_val = self.PIDsteer(steer_setpoint)

                if r_dist > 0: # We're going forward
                    if PID_val >= 0:
                        self.m.signal(self.m.STEER_LEFT, PID_val)
                    else:
                        self.m.signal(self.m.STEER_RIGHT, 0.0 - PID_val)
                else: # We're going in reverse
                    if PID_val >= 0:
                        self.m.signal(self.m.STEER_RIGHT, PID_val)
                    else:
                        self.m.signal(self.m.STEER_LEFT, 0.0 - PID_val)
                    

                num_iter = num_iter - 1
                if meter < 0.0 and r_dist > -0.1:
                    arrived = True
                    self.logger.info("Arrived going backwards")
                if meter >= 0.0 and r_dist < 0.1:
                    arrived = True
                    self.logger.info("Arrived going forwards")
                if num_iter <= 0:
                    self.logger.info("Arrived after bailout")
                    arrived = True
                    failure = True
            self.m.signal(self.m.STOP, 0)

            # Allow .2 sec to coast to a halt
            num_extra = int(self.Fs / 5)
            for i in range(num_extra):
                self.get_inertial_measurements(self.m)                
                r_dist = meter - self.inertial_data[0, self.dist_x]
                
            if self.collect_PID_data:
                self.dump_PID_store(self.PID_store)

            self.logger.info("Tracker.move_dist(%3.3f) done: r_dist: %3.3f" % (meter, r_dist))

        except:
            self.m.signal(self.m.STOP, 0)
            self.logger.error("Tracker.move_dist(%3.3f) failed" % (meter))
            self.logger.error(sys.exc_info())
            print("ERROR: Tracker.move_dist(%3.3f) failed" % (meter))
            print(sys.exc_info())
            failure = True
        if failure:
            return -998
        else:
            return r_dist

    def PIDsteer(self, setpoint):
        """ Calculate the error value to apply to the steering to keep the robot
        tracking in a straight line.
        Return a Controller Output (CO) in the range -100 to +100
        """
        # Calculate values for the PID regulator
        P = self.inertial_data[0, self.angle_z]
        max_p = max(self.inertial_data[0, self.steer_P], self.inertial_data[1, self.steer_P])
        min_p = min(self.inertial_data[0, self.steer_P], self.inertial_data[1, self.steer_P])
        I = ((max_p - min_p) / 2 * self.Ta) + (min_p * self.Ta)
        D = self.inertial_data[0, self.gyr_z]

        # Adjust PID tuning
        P = P * self.Kp_steer
        I = I * self.Ki_steer
        D = D * self.Kd_steer

        self.inertial_data[0, self.steer_P] = P
        self.inertial_data[0, self.steer_I] = I
        self.inertial_data[0, self.steer_D] = D
        
        errval = P + I + D
        # Calculate and adjust the Controller Output
        CO = setpoint - errval
        if CO < -100:
            CO = -100.0
        elif CO > 100:
            CO = 100.0
        # print("P: %2.3f I: %2.3f D: %2.3f E: %2.3f CO: %2.3f" % (P, I, D, errval, CO))
        return CO

    def PIDdist(self, setpoint):
        """ The setpoint is the remaining distance to travel. Positive (forward) or negative (reverse)
        """
        if setpoint >= 0:
            P = (setpoint * self.Kp_dist) + self.s.motor_min_power
        else:
            P = (setpoint * self.Kp_dist) - self.s.motor_min_power
            
        self.inertial_data[0, self.dist_P] = P
        D = self.inertial_data[0, self.vel_x] * self.Kd_dist
        self.inertial_data[0, self.dist_D] = D
        max_p = max(self.inertial_data[0, self.dist_x], self.inertial_data[1, self.dist_x])
        min_p = min(self.inertial_data[0, self.dist_x], self.inertial_data[1, self.dist_x])
        I = ((max_p - min_p) * self.Ta + (min_p * self.Ta)) * self.Ki_dist
        self.inertial_data[0, self.dist_I] = I

        power = P + I + D
        if power > 100:
            power = 100
        elif power < -100:
            power = -100
            
        return power
            
    def get_inertial_measurements(self, motor):
        """ Sample data from the inertial navigation sensors and prepare the values for the PID regulators.
            Acceleration data and gyro data is sampled at Fs
            Operates directly on the class buffer inertial_data
        """

        if not self.first_run:
            # See if we are dumping the array to file
            if self.collect_PID_data:
                self.PID_store[self.dbg_cnt] = np.copy(self.inertial_data[0])
                self.dbg_cnt = self.dbg_cnt + 1
                if self.dbg_cnt >= 1999:
                    self.dump_PID_store(self.PID_store)

            # Rotate the inertial_data array so that index 0 is ready to accept new values
            for i in range(self.BUFSZ - 1, 0, -1):
                self.inertial_data[i] = self.inertial_data[i - 1]


        # Maintain correct sampling interval
        self.t_before_sleep = time.time()
        if not self.first_run:
            difftime = self.t_before_sleep - self.inertial_data[1, self.tstamp]
        else:
            difftime = 0.0
        sleeptime = self.Ts - difftime - self.sensor_read_time
        if sleeptime > 0.001 :
            time.sleep(sleeptime) # maintain sampling rate
        self.inertial_data[0, self.tsleep] = sleeptime
        # Read the raw data from the inertial navigation sensors
        acc = self.a.getAxes()
        gyr = self.g.read()
        self.t_prev = self.t_now
        self.t_now = time.time()

        # Calculate actual time elapsed since previous sample, Ta
        if not self.first_run:
            self.Ta = self.t_now - self.t_prev
        else:
            self.Ta = 0.0
            self.first_run = False
        self.inertial_data[0, self.samplingtime] = self.Ta

        

        # Store the most recent raw data at index 0 in the inertial_data buffer
        self.inertial_data[0, self.tstamp] = self.t_now
        
        self.inertial_data[0, self.acc_x] = acc[0] # - self.x_offset
        self.inertial_data[0, self.acc_y] = acc[1] # - self.y_offset
        self.inertial_data[0, self.acc_z] = acc[2] # - self.z_offset
        self.inertial_data[0, self.gyr_x] = gyr[0]
        self.inertial_data[0, self.gyr_y] = gyr[1]
        self.inertial_data[0, self.gyr_z] = gyr[2]

        # self.hanning_filter()
        self.lowpass_filter()
        
        instant_gyro = self.inertial_data[0, self.gyr_z] # It's the rotation about the Z axis we're using

        self.inertial_data[0, self.angle_x] = self.inertial_data[1, self.angle_x] + (self.inertial_data[0, self.gyr_x] * self.Ta)
        self.inertial_data[0, self.angle_y] = self.inertial_data[1, self.angle_y] + (self.inertial_data[0, self.gyr_y] * self.Ta)
        self.inertial_data[0, self.angle_z] = self.inertial_data[1, self.angle_z] + (self.inertial_data[0, self.gyr_z] * self.Ta)

        # Use unfiltered acceleration values
        self.inertial_data[0, self.vel_x] = self.inertial_data[1, self.vel_x] + (self.inertial_data[0, self.acc_x] * self.Ta)
        self.inertial_data[0, self.vel_y] = self.inertial_data[1, self.vel_y] + (self.inertial_data[0, self.acc_y] * self.Ta)
        self.inertial_data[0, self.vel_z] = self.inertial_data[1, self.vel_z] + (self.inertial_data[0, self.acc_z] * self.Ta)

        # Use filtered acceleration values
        # self.inertial_data[0, self.vel_x] = self.inertial_data[1, self.vel_x] + (self.inertial_data[0, self.flt_acc_x] * self.Ta)
        # self.inertial_data[0, self.vel_y] = self.inertial_data[1, self.vel_y] + (self.inertial_data[0, self.flt_acc_y] * self.Ta)
        # self.inertial_data[0, self.vel_z] = self.inertial_data[1, self.vel_z] + (self.inertial_data[0, self.flt_acc_z] * self.Ta)

        self.inertial_data[0, self.dist_x] = self.inertial_data[1, self.dist_x] + (self.inertial_data[0, self.vel_x] * self.Ta)
        self.inertial_data[0, self.dist_y] = self.inertial_data[1, self.dist_y] + (self.inertial_data[0, self.vel_y] * self.Ta)
        self.inertial_data[0, self.dist_z] = self.inertial_data[1, self.dist_z] + (self.inertial_data[0, self.vel_z] * self.Ta)


    def hanning_filter(self):
        """
        Moving average filter with double weight on present value. Filter the x, y, z acceleration values
        """
        
        filtered_value_x = 2 * (self.inertial_data[0, self.acc_x] - self.x_offset)
        filtered_value_y = 2 * (self.inertial_data[0, self.acc_y] - self.y_offset)
        filtered_value_z = 2 * (self.inertial_data[0, self.acc_z] - self.z_offset)
        for i in range(1, self.BUFSZ):
            filtered_value_x = filtered_value_x + self.inertial_data[i, self.flt_acc_x]
            filtered_value_y = filtered_value_y + self.inertial_data[i, self.flt_acc_y]
            filtered_value_z = filtered_value_z + self.inertial_data[i, self.flt_acc_z]
        self.inertial_data[0, self.flt_acc_x] = filtered_value_x / (self.BUFSZ + 1)
        self.inertial_data[0, self.flt_acc_y] = filtered_value_y / (self.BUFSZ + 1)
        self.inertial_data[0, self.flt_acc_z] = filtered_value_z / (self.BUFSZ + 1)


    def lowpass_filter(self):
         self.inertial_data[0, self.flt_acc_x] = (self.inertial_data[0, self.acc_x] * self.alpha) + (self.inertial_data[1, self.flt_acc_x] * (1.0 - self.alpha))
         self.inertial_data[0, self.flt_acc_y] = (self.inertial_data[0, self.acc_y] * self.alpha) + (self.inertial_data[1, self.flt_acc_y] * (1.0 - self.alpha))
         self.inertial_data[0, self.flt_acc_z] = (self.inertial_data[0, self.acc_z] * self.alpha) + (self.inertial_data[1, self.flt_acc_z] * (1.0 - self.alpha))
         

    def set_heading(self, hdg):
        """ Rotate the robot to the given ablolute heading
        """
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
        """ Calculate how many degrees are left to rotate
        """
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
        giveup = 500 # If we haven't completed the turn in 5 sec, give up
        self.buf_reset()
        # self.Ta = self.Ts    # Reset the actual sample time to avoid unexpected startup conditions
        self.first_run = True
        completed_angle = 0.0
        while rel_angle < -180.0:
            rel_angle = rel_angle + 360.0
        while rel_angle > 180.0:
            rel_angle = rel_angle - 360.0
            
        # Empirically add 10% to the turn angle. Due to gyro ?
        rel_angle = rel_angle + ((rel_angle / 100) * 10)

        turn_angle = rel_angle
        self.t_now = time.time()
        while (turn_angle > 1.0 or turn_angle < -1.0) and giveup > 0:
            self.get_inertial_measurements(self.m)
            PID = self.PIDturn(turn_angle)
            if PID >= 0.0:
                self.m.signal(self.m.TURN_RIGHT, PID)
            else:
                self.m.signal(self.m.TURN_LEFT, 0.0 - PID)
            completed_angle = self.inertial_data[0, self.angle_z]
            turn_angle = rel_angle - completed_angle
            giveup = giveup - 1
                        
        self.m.signal(self.m.STOP, 0.0)

        # Allow .2 sec to coast to a halt
        num_extra = int(self.Fs / 5)
        for i in range(num_extra):
            self.get_inertial_measurements(self.m)                
            completed_angle = self.inertial_data[0, self.angle_z]
            turn_angle = rel_angle - completed_angle
        
        if self.collect_PID_data:
            self.dump_PID_store(self.PID_store)

        return turn_angle

    def PIDturn(self, turn_setpoint):
        """ Calculate the power to be applied to turning left or right while
            orienting the robot
            """
        # P = self.inertial_data[0, self.angle_z] * self.Kp_turn
        P = turn_setpoint * self.Kp_turn
        self.inertial_data[0, self.rot_P] = P
        max_a = max(self.inertial_data[0, self.rot_P], self.inertial_data[1, self.rot_P])
        min_a = min(self.inertial_data[0, self.rot_P], self.inertial_data[1, self.rot_P])
        I = ((((max_a - min_a) / 2) * self.Ta) + (min_a * self.Ta)) * self.Ki_turn
        D = self.inertial_data[0, self.gyr_z] * self.Kd_turn
        self.inertial_data[0, self.rot_I] = I
        self.inertial_data[0, self.rot_D] = D
        errval = P + I + D
        CO = turn_setpoint - errval
        if CO < -100.0:
            CO = -100.0
        if CO > 100.0:
            CO = 100.0

        return CO
        
    def calc_bailout_dist(self, meters):
        """
        Calculate the maximum number of samples required to achieve goal
        Assume an average speed of 0.25 m/s
        """
        avg_speed = 0.25  # m/s. Perhaps make this a property in sysprops?
        if meters < 0.0:
            meters = 0.0 - meters
        num_sec = meters / avg_speed
        iterations = self.Fs * num_sec
        iterations = iterations * 2 # Add 100%...
        return int(iterations)
        
    def turn(self, angle, hdg):
        """ Adjust the relative power of the motors while running forward
        """
        for i in range(20):
            adj_accel_data, adj_gyro_data = self.get_inertial_measurements(self.m)
            self.m.steer(self.PIDsteer(angle))

    def buf_reset(self):
        for i in range(self.BUFSZ):
            for j in range(self.inertial_data[0].size):
                self.inertial_data[i, j] = 0.0
                
            
    def dump_PID_store(self, PID_store):
        hdr = ("tstamp, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, angle_x, angle_y, angle_z, "
               "vel_x, vel_y, vel_z, steer_P, steer_I, steer_D, dist_P, dist_I, "
               "dist_D, rot_P, rot_I, rot_D, samplingtime, tsleep, dist_x, dist_y, dist_z, "
               "flt_acc_x, flt_acc_y, flt_acc_z")
        path = os.path.join(self.s.logdir, "PID_data_log_" + str(time.time()) + ".csv")
        np.savetxt(path, PID_store[0:self.dbg_cnt], "%3.3f", header=hdr, delimiter=',')
        self.dbg_cnt = 0

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
        """ Check return value from the specified address to veify that it is actually
            a ITG3205 chip at that address
        """
        value = self.bus.read_byte_data(self.address, 0)
        print("Gyro self test %4X" % (value))
        if value == 0x68:
            return True
        return False

    def read(self):
        """ Read data back from 0x1D(29), 6 bytes
            X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
            Returns instantaneous rotation around the X, Y and Z axis in degrees per second.
            """
        
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
    """ Interface to the ADXL345 accelerometer chip on the 9-degrees of freedom
    Subassembly (GY-85).

    """
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
        self.setBandwidthRate(self.BW_RATE_200HZ)
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

    """ HMC5888L Magnetometer (Digital Compass) wrapper class.
        The chip is located on the GY-85 subassembly and is accessed using I2C 
        Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
        but uses smbus rather than quick2wire and sets some different init
        params.

        originally class hmc5883l:

        Norway: gauss = 0.507, declination = 0.88 deg
    """

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
        self.bus.write_byte_data(self.address, 0x00, 0x10) # 1 Average, 15 Hz, normal measurement
        # self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x01, 0x20) # Scale
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

        alt_x_axis = np.int32((x_axis_msb * 256) + x_axis_lsb)
        alt_y_axis = np.int32((y_axis_msb * 256) + y_axis_lsb)
        alt_z_axis = np.int32((z_axis_msb * 256) + z_axis_lsb)

        x_axis = self.twos_complement(x_axis_msb << 8 | x_axis_lsb, 16)
        y_axis = self.twos_complement(y_axis_msb << 8 | y_axis_lsb, 16)
        z_axis = self.twos_complement(z_axis_msb << 8 | z_axis_lsb, 16)

        raw_heading = math.atan2(y_axis * self.__scale, x_axis * self.__scale)

        print("X MSB %4X X LSB %4X" % (x_axis_msb, x_axis_lsb))
        print("Y MSB %4X Y LSB %4X" % (y_axis_msb, y_axis_lsb))
        print("Z MSB %4X Z LSB %4X" % (z_axis_msb, z_axis_lsb))
        print("X %4d Y %4d Z %4d" % (x_axis, y_axis, z_axis))
        print("Alt X %4d Y %4d Z %4d" % (alt_x_axis, alt_y_axis, alt_z_axis))
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

    def heading_rad(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x)
        headingRad += self.__declination

        # Correct for reversed heading
        # if headingRad < 0:
        #     headingRad += 2 * math.pi

        # Check for wrap and compensate
        # elif headingRad > 2 * math.pi:
        #     headingRad -= 2 * math.pi
        return headingRad
        

    def heading(self):

        # Convert to degrees from radians
        headingDeg = self.heading_rad() * 180 / math.pi
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


