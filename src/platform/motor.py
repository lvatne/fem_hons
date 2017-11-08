import RPi.GPIO as GPIO  
import time  
from RPIO import PWM
import logging


class Motor:
    """
    **NOT IN USE IN VERSION 1.0**
    
    IO interface to the propulsion motor(s)
    Note that we use the BCM scheme for pin numbering.
    This due to BCM being a requirement of the RPIO module as per 12/9-2016

    The following need calibration:
    TURN_FACTOR
    NEUTRAL_TURN_FACTOR
    ..and possibly also the speed values used in the 'turn..' methods.
    
    Attributes:
        None.
    """
    LEFT_FORWARD = 20                             
    LEFT_REVERSE = 21                             
    RIGHT_FORWARD = 5                             
    RIGHT_REVERSE = 6                             
    TURN_FACTOR = 70.2
    NEUTRAL_TURN_FACTOR = 35.1

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LEFT_FORWARD, GPIO.OUT)
        GPIO.setup(self.LEFT_REVERSE, GPIO.OUT)
        GPIO.setup(self.RIGHT_FORWARD, GPIO.OUT)
        GPIO.setup(self.RIGHT_REVERSE, GPIO.OUT)
        
        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)  
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)  

        self.servo = PWM.Servo(14, 20000) #using DMA ch 14, pulse width 20000 uS (20 MSec)
        self.logger = logging.getLogger('propulsion')
        hdlr = logging.FileHandler('/var/tmp/propulsion.log')
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.INFO)


    def calc_subcycle(self, pct):
        """
            Keyword arguments:
            pct: percentage power 0 - 100.

            Effective range is 20 - 95. If more than 95, we set full power.
            If less than 20 we set engine stop.
        """
        
        if pct >= 95:
            subcycle_time = 20000
        elif pct <= 20:
            subcycle_time = 0
        else:
            subcycle_time = pct * 20000 / 100
        tmp_cycle = int(subcycle_time / 20)
        subcycle_time = tmp_cycle * 20   # Must be divisible by 20
        return int(subcycle_time)

    def stop(self) :
        logger.info('stopping...')
        servo.stop_servo(self.LEFT_FORWARD)
        servo.stop_servo(self.LEFT_REVERSE)
        servo.stop_servo(self.RIGHT_FORWARD)
        servo.stop_servo(self.RIGHT_REVERSE)

        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)  
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)
        logger.info('..stopped')
        

    def left_fwd(self, speed) :
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        subcycle_time = self.calc_subcycle(speed)
        self.servo.set_servo(self.LEFT_FORWARD, subcycle_time)

    def left_rev(self, speed) :
        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)  
        subcycle_time = self.calc_subcycle(speed)
        self.servo.set_servo(LEFT_REVERSE, subcycle_time)
        
    def right_fwd(self, speed) :
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)  
        subcycle_time = self.calc_subcycle(speed)
        self.servo.set_servo(self.RIGHT_FORWARD, subcycle_time)
        
    def right_rev(self, speed) :
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        subcycle_time = self.calc_subcycle(speed)
        self.servo.set_servo(RIGHT_REVERSE, subcycle_time)

    def turn_calc_sec(self, dega) :
        sec = dega / self.TURN_FACTOR
        return sec

    def neutral_calc(self, dega) :
        sec = dega / self.NEUTRAL_TURN_FACTOR
        return sec


     # High level methods
     
    def forward(self, speed, sec) :
         """
             Go forward with specified speed for the specified number
             of seconds.
             Keyword arguments:
             speed: Percentage power (0-100)
             sec: number of seconds to run
         """
        
         self.logger.info("forward({}, {})".format(speed, sec))
         self.left_fwd(speed)
         self.right_fwd(speed)
         sleep(sec)
         self.stop()
         self.logger,info("forward() done")

    def reverse(self, speed, sec) :
         """
             Go backwards with specified speed for the specified number
             of seconds.
             Keyword arguments:
             speed: Percentage power (0-100)
             sec: number of seconds to run
         """
        
        self.logger.info("reverse({}, {})",format(speed, sec))
        self.right_rev(speed)
        self.left_rev(speed)
        sleep(sec)
        self.stop()
        self.logger.info("reverse() done")


    def right_turn(self, dega) :
        self.logger.info("right_turn({})".format(dega))
        sec = self.turn_calc_sec(dega)
        self.stop()
        self.left_fwd(50)
        sleep(sec)
        self.stop()
        self.logger.info("right_turn() done")

    def left_turn(self, dega) :
        self.logger.info("left_turn({})".format(dega))
        sec = self.turn_calc(dega)
        self.stop()
        self.right_fwd(50)
        sleep(sec)
        self.stop()
        self.logger.info("left_turn() done")

    def right_neutral_turn(self, dega) :
        self.logger.info("right_neutral_turn({})".format(dega))
        sec = self.neutral_calc(dega)
        self.stop()
        self.left_fwd(30)
        self.right_rev(30)
        sleep(sec)
        self.stop()
        self.logger.info("right_neutral_turn() done")

    def left_neutral_turn(self, dega) :
        self.logger.info("left_neutral_turn({})".format(dega))
        sec = self.neutral_calc(dega)
        self.stop()
        self.right_fwd(30)
        self.left_rev(30)
        sleep(sec)
        self.stop()
        self.logger.info("left_neutral_turn() done")
        
        
        
        
