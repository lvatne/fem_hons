import numpy as np
import smbus
import math
import logging
import sysprops
import time
import sys
import os
import logging


# coding: utf-8
## @package MPU9250
#  This is a FaBo9Axis_MPU9250 library for the FaBo 9AXIS I2C Brick.
#
#  http://fabo.io/202.html
#
#  Released under APACHE LICENSE, VERSION 2.0
#
#  http://www.apache.org/licenses/
#
#  FaBo <info@fabo.io>

import smbus
import time

## MPU9250 Default I2C slave address
SLAVE_ADDRESS        = 0x68
## AK8963 I2C slave address
AK8963_SLAVE_ADDRESS = 0x0C
## Device id
DEVICE_ID            = 0x71

''' MPU-9250 Register Addresses '''
## sample rate driver
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
LP_ACCEL_ODR   = 0x1E
WOM_THR        = 0x1F
FIFO_EN        = 0x23
I2C_MST_CTRL   = 0x24
I2C_MST_STATUS = 0x36
INT_PIN_CFG    = 0x37
INT_ENABLE     = 0x38
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43

I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL    = 0x69
USER_CTRL          = 0x6A
PWR_MGMT_1         = 0x6B
PWR_MGMT_2         = 0x6C
FIFO_R_W           = 0x74
WHO_AM_I           = 0x75

## Gyro Full Scale Select 250dps
GFS_250  = 0x00
## Gyro Full Scale Select 500dps
GFS_500  = 0x01
## Gyro Full Scale Select 1000dps
GFS_1000 = 0x02
## Gyro Full Scale Select 2000dps
GFS_2000 = 0x03
## Accel Full Scale Select 2G
AFS_2G   = 0x00
## Accel Full Scale Select 4G
AFS_4G   = 0x01
## Accel Full Scale Select 8G
AFS_8G   = 0x02
## Accel Full Scale Select 16G
AFS_16G  = 0x03

# AK8963 Register Addresses
AK8963_ST1        = 0x02
AK8963_MAGNET_OUT = 0x03
AK8963_CNTL1      = 0x0A
AK8963_CNTL2      = 0x0B
AK8963_ASAX       = 0x10

# CNTL1 Mode select
## Power down mode
AK8963_MODE_DOWN   = 0x00
## One shot data output
AK8963_MODE_ONE    = 0x01

## Continous data output 8Hz
AK8963_MODE_C8HZ   = 0x02
## Continous data output 100Hz
AK8963_MODE_C100HZ = 0x06

# Magneto Scale Select
## 14bit output
AK8963_BIT_14 = 0x00
## 16bit output
AK8963_BIT_16 = 0x01

_mpu9250 = None

## smbus
bus = smbus.SMBus(1)

## MPU9250 I2C Controll class
class MPU9250:

    ## Constructor
    #  @param [in] address MPU-9250 I2C slave address default:0x68
    def __init__(self, address=SLAVE_ADDRESS):
        self.address = address
        self.configMPU9250(GFS_250, AFS_2G)
        self.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)

    ## Search Device
    #  @param [in] self The object pointer.
    #  @retval true device connected
    #  @retval false device error
    def searchDevice(self):
        who_am_i = bus.read_byte_data(self.address, WHO_AM_I)
        if(who_am_i == DEVICE_ID):
            return True
        else:
            return False

    ## Configure MPU-9250
    #  @param [in] self The object pointer.
    #  @param [in] gfs Gyro Full Scale Select(default:GFS_250[+250dps])
    #  @param [in] afs Accel Full Scale Select(default:AFS_2G[2g])
    def configMPU9250(self, gfs, afs):
        if gfs == GFS_250:
            self.gres = 250.0/32768.0
        elif gfs == GFS_500:
            self.gres = 500.0/32768.0
        elif gfs == GFS_1000:
            self.gres = 1000.0/32768.0
        else:  # gfs == GFS_2000
            self.gres = 2000.0/32768.0

        if afs == AFS_2G:
            self.ares = 2.0/32768.0
        elif afs == AFS_4G:
            self.ares = 4.0/32768.0
        elif afs == AFS_8G:
            self.ares = 8.0/32768.0
        else: # afs == AFS_16G:
            self.ares = 16.0/32768.0

        # sleep off
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # auto select clock source
        bus.write_byte_data(self.address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # DLPF_CFG
        bus.write_byte_data(self.address, CONFIG, 0x03)
        # sample rate divider
        bus.write_byte_data(self.address, SMPLRT_DIV, 0x04)
        # gyro full scale select
        bus.write_byte_data(self.address, GYRO_CONFIG, gfs << 3)
        # accel full scale select
        bus.write_byte_data(self.address, ACCEL_CONFIG, afs << 3)
        # A_DLPFCFG
        bus.write_byte_data(self.address, ACCEL_CONFIG_2, 0x03)
        # BYPASS_EN
        bus.write_byte_data(self.address, INT_PIN_CFG, 0x02)
        time.sleep(0.1)

    ## Configure AK8963
    #  @param [in] self The object pointer.
    #  @param [in] mode Magneto Mode Select(default:AK8963_MODE_C8HZ[Continous 8Hz])
    #  @param [in] mfs Magneto Scale Select(default:AK8963_BIT_16[16bit])
    def configAK8963(self, mode, mfs):
        if mfs == AK8963_BIT_14:
            self.mres = 4912.0/8190.0
        else: #  mfs == AK8963_BIT_16:
            self.mres = 4912.0/32760.0

        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)

        # set read FuseROM mode
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x0F)
        time.sleep(0.01)

        # read coef data
        data = bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_ASAX, 3)

        self.magXcoef = (data[0] - 128) / 256.0 + 1.0
        self.magYcoef = (data[1] - 128) / 256.0 + 1.0
        self.magZcoef = (data[2] - 128) / 256.0 + 1.0

        # set power down mode
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00)
        time.sleep(0.01)

        # set scale&continous mode
        bus.write_byte_data(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, (mfs<<4|mode))
        time.sleep(0.01)

    ## brief Check data ready
    #  @param [in] self The object pointer.
    #  @retval true data is ready
    #  @retval false data is not ready
    def checkDataReady(self):
        drdy = bus.read_byte_data(self.address, INT_STATUS)
        if drdy & 0x01:
            return True
        else:
            return False

    ## Read accelerometer
    #  @param [in] self The object pointer.
    #  @retval x : x-axis data
    #  @retval y : y-axis data
    #  @retval z : z-axis data
    def readAccel(self):
        data = bus.read_i2c_block_data(self.address, ACCEL_OUT, 6)
        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.ares, 3)
        y = round(y*self.ares, 3)
        z = round(z*self.ares, 3)
        #return {"x":x, "y":y, "z":z}
        return np.array([x, y, z])


    ## Read gyro
    #  @param [in] self The object pointer.
    #  @retval x : x-gyro data
    #  @retval y : y-gyro data
    #  @retval z : z-gyro data
    def readGyro(self):
        data = bus.read_i2c_block_data(self.address, GYRO_OUT, 6)

        x = self.dataConv(data[1], data[0])
        y = self.dataConv(data[3], data[2])
        z = self.dataConv(data[5], data[4])

        x = round(x*self.gres, 3)
        y = round(y*self.gres, 3)
        z = round(z*self.gres, 3)

        # return {"x":x, "y":y, "z":z}
        return np.array([x, y, z])


    ## Read magneto
    #  @param [in] self The object pointer.
    #  @retval x : X-magneto data
    #  @retval y : y-magneto data
    #  @retval z : Z-magneto data
    def readMagnet(self):
        x=0
        y=0
        z=0

        # check data ready
        drdy = bus.read_byte_data(AK8963_SLAVE_ADDRESS, AK8963_ST1)
        if drdy & 0x01 :
            data = bus.read_i2c_block_data(AK8963_SLAVE_ADDRESS, AK8963_MAGNET_OUT, 7)

            # check overflow
            if (data[6] & 0x08)!=0x08:
                x = self.dataConv(data[0], data[1])
                y = self.dataConv(data[2], data[3])
                z = self.dataConv(data[4], data[5])

                x = round(x * self.mres * self.magXcoef, 3)
                y = round(y * self.mres * self.magYcoef, 3)
                z = round(z * self.mres * self.magZcoef, 3)

        #return {"x":x, "y":y, "z":z}
        return (x, y, z)

    ## Read temperature
    #  @param [out] temperature temperature(degrees C)
    def readTemperature(self):
        data = bus.read_i2c_block_data(self.address, TEMP_OUT, 2)
        temp = self.dataConv(data[1], data[0])

        temp = round((temp / 333.87 + 21.0), 3)
        return temp

    ## Data Convert
    # @param [in] self The object pointer.
    # @param [in] data1 LSB
    # @param [in] data2 MSB
    # @retval Value MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value




class Gyro_GY9250:
    """ Interface to the MPU-9250 gyro chip on the 9-degrees of freedom
    Subassembly (GY-9250).

    
    Positive Z values: Counter-clockwise rotation
    Negative Z values: clockwise rotation
    """
    
    def __init__(self, address = 0x68):
        global _mpu9250
        if _mpu9250 is None:
            _mpu9250 = MPU9250()
            _mpu9250.configMPU9250(GFS_250, AFS_2G)
            _mpu9250.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)
        self.mpu9250 = _mpu9250

            
    def self_test(self):
        """
         Check return value from the specified address to veify that it is actually
            a MPU9250chip at that address
        """
        return self.mpu9250.searchDevice()

    def read(self):
        """Returns instantaneous rotation around the X, Y and Z axis in degrees per second.
        """
        return self.mpu9250.readGyro()    
        




class Accelerometer_GY9250:
    """ Interface to the accelerometer chip on the 9-degrees of freedom
    Subassembly (GY-9250).
    """
    

    def __init__(self):
        global _mpu9250
        if _mpu9250 is None:
            _mpu9250 = MPU9250()
            _mpu9250.configMPU9250(GFS_250, AFS_2G)
            _mpu9250.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)
        self.mpu9250 = _mpu9250

    
    # returns the current reading from the sensor for each axis
    #
    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs
    def getAxes(self, gforce = False):
        return self.mpu9250.readAccel()

    def read_raw(self):
        return self.mpu9250.readAccel()



class Compass_GY9250:

    """HMC5888L Magnetometer (Digital Compass) wrapper class.
        The chip is located on the GY-9250 subassembly and is accessed using I2C 
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
        global _mpu9250
        if _mpu9250 is None:
            _mpu9250 = MPU9250()
            _mpu9250.configMPU9250(GFS_250, AFS_2G)
            _mpu9250.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)
        self.mpu9250 = _mpu9250
        
        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
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
        return self.mpu9250.searchDevice()

    def reset(self):
        self.bus.write_byte_data(self.address, 0x02, 0x03) # Set in idle mode
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def test_read(self):
        (x_axis, y_axis, z_axis) = self.mpu9250.readMagnet()

        raw_heading = math.atan2(y_axis * self.__scale, x_axis * self.__scale)

        print("Raw heading %2.4f rad %3.2d deg" % (raw_heading, raw_heading * (180.0 / math.pi)))
        
        
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
        return self.mpu9250.readMagnet()

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





if __name__ == '__main__':
    mpu9250 = MPU9250()
    if mpu9250.searchDevice():
        print("Found MPU9250")
        mpu9250.configMPU9250(GFS_250, AFS_2G)
        mpu9250.configAK8963(AK8963_MODE_C8HZ, AK8963_BIT_16)

        if mpu9250.checkDataReady():
            print('Data ready')
            print('Magnet:')  
            for i in range(10):
                print(mpu9250.readMagnet())
                time.sleep(0.1)
            print('Acceleration:')
            for i in range(10):
                print(mpu9250.readAccel())
                time.sleep(0.1)
            print('Gyro:')
            for i in range(10):
                print(mpu9250.readGyro())
                time.sleep(0.1)
            print('Temperature:')
            for i in range(10):
                print(mpu9250.readTemperature())
                time.sleep(0.1)
        else:
            print('Data NOT ready')
    else:
        print("MPU9250 not found")

    print('Gyro_9250()')
    gyro = Gyro_GY9250()
    for i in range(10):
        print(gyro.read())
        time.sleep(0.1)

    print('Accelerometer_GY9250()')
    accel = Accelerometer_GY9250()
    for i in range(10):
        print(accel.getAxes())
        time.sleep(0.1)

    print('Compass_GY9250()')
    compass = Compass_GY9250()
    for i in range(10):
        print(compass.test_read())

 
