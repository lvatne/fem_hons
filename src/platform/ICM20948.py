import numpy as np
import smbus
import math
import logging
import struct
import time
import sys
import os
import logging
import sysprops



class Gyro_ICM20948:
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



class Accelerometer_ICM20948:
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


class Compass_ICM20948:

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

class Pressure_ICM20948:
    def __init__(self, address=0x00):
        self.address = address


if __name__ == "__main__":
    gyro = Gyro_ICM20948()
    accel = Accelerometer_ICM20948()
    comp = Compass_ICM20948()
    press = Pressure_ICM20948()
    
    
