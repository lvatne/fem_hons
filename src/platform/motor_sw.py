import RPi.GPIO as GPIO  
import time
import math
#from RPIO import PWM
import logging
import sysprops
import os


class Motor_sw:
    """ IO interface to the propulsion motor(s)
    Note that we use the BCM scheme for pin numbering.
    This due to BCM being a requirement of the RPIO module as per 12/9-2016

    The following need calibration:
    TURN_FACTOR
    NEUTRAL_TURN_FACTOR
    ..and possibly also the speed values used in the 'turn..' methods.
    
    Attributes:
        None.
    """
    # GPIO pins for servos
    LEFT_FORWARD = 6                             
    LEFT_REVERSE = 5                             
    RIGHT_FORWARD = 21                             
    RIGHT_REVERSE = 20                       
    TURN_FACTOR = 70.2
    NEUTRAL_TURN_FACTOR = 35.1
    ROLL_UP_TIME = 0.05

    # States
    Stopped = 0
    Turning_Right = 1
    Turning_Left = 2
    Running_Fwd = 3
    Running_Rev = 4
    Error_State = 5

    state_names = ["Stopped = 0",  "Turning_Right = 1", "Turning_Left = 2",
                   "Running_Fwd = 3", "Running_Rev = 4", "Error_State = 5"]

    # Signals
    STOP = 10
    TURN_RIGHT = 11
    TURN_LEFT = 12
    STEER_RIGHT = 13
    STEER_LEFT = 14
    RUN_FWD = 15
    RUN_REV = 16

    signal_names = [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ",
                    "STOP", "TURN_RIGHT", "TURN_LEFT",
                    "STEER_RIGHT", "STEER_LEFT", "RUN_FWD", "RUN_REV"]

    def __init__(self):
        # GPIO.cleanup()
        # GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LEFT_FORWARD, GPIO.OUT)
        GPIO.setup(self.LEFT_REVERSE, GPIO.OUT)
        GPIO.setup(self.RIGHT_FORWARD, GPIO.OUT)
        GPIO.setup(self.RIGHT_REVERSE, GPIO.OUT)
        
        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)  
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)

        self.lf_servo = GPIO.PWM(self.LEFT_FORWARD, 20)
        self.lr_servo = GPIO.PWM(self.LEFT_REVERSE, 20)
        self.rf_servo = GPIO.PWM(self.RIGHT_FORWARD, 20)
        self.rr_servo = GPIO.PWM(self.RIGHT_REVERSE, 20)

        self.power = 0.0
        self.lPower = 0.0
        self.rPower = 0.0
        self.dbg_cnt = 0

        # Initialize the state machine
        self.state = self.Stopped
        self.prev_state = self.Stopped
        self.curr_signal = self.STOP
        self.prev_signal = self.STOP
        self.logger = logging.getLogger('propulsion')        

        try:
            robohome = os.environ['ROBOHOME']
        except KeyError:
            print("Error in installation. $ROBOHOME does not exist (motor_sw)")
            self.logger.error("Error in installation. $ROBOHOME does not exist (motor_sw)")
            raise
        logdir = os.path.join(robohome, "log")
        hdlr = logging.FileHandler(os.path.join(logdir, "propulsion.log"))
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        self.logger.addHandler(hdlr) 
        self.logger.setLevel(logging.WARNING)
        self.logger.info("--------------+++--------------")
        s = sysprops.SysProps()
        self.MIN_PWR = s.motor_min_power


    # Entry point for the motor state machine
    def signal(self, sign, pwr):
        self.prev_signal = self.curr_signal
        self.curr_signal = sign
        self.logger.info("Motor: Received %s in state %s" %
                         (self.signal_names[sign], self.state_names[self.state]))
        if sign == self.STOP:
            self.stop()
        elif sign == self.TURN_RIGHT:
            self.turn_right(pwr)
        elif sign == self.TURN_LEFT:
            self.turn_left(pwr)
        elif sign == self.STEER_RIGHT:
            self.steer_right(pwr)
        elif sign == self.STEER_LEFT:
            self.steer_left(pwr)
        elif sign == self.RUN_FWD:
            self.run_fwd(pwr)
        elif sign == self.RUN_REV:
            self.run_rev(pwr)
        else:
            self.logger.error("ERROR: Motor state machine error. Received unknown signal %d" % (sign))

    # To be used to come out of an unknown state when debugging
    def force_stop(self):
        self.lf_servo.stop()
        self.rr_servo.stop()
        self.rf_servo.stop()
        self.lr_servo.stop()
        self.state = self.Stopped
        self.power = 0.0
        self.rPower = 0.0
        self.lPower = 0.0
        self.logger.warning('Motor Force Stopped')
        
    
    # Part of the state machine. Invoked on receiving the STOP signal
    def stop(self):
        self.logger.info('stopping from state ' + self.state_names[self.state])
        if self.state == self.Stopped:
            new_state = self.Stopped
        elif self.state == self.Turning_Right:
            self.lf_servo.stop()
            self.rr_servo.stop()
            new_state = self.Stopped
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            self.lr_servo.stop()
            new_state = self.Stopped
        elif self.state == self.Running_Fwd:
            self.lf_servo.stop()
            self.rf_servo.stop()
            new_state = self.Stopped
        elif self.state == self.Running_Rev:
            self.lr_servo.stop()
            self.rr_servo.stop()
            new_state = self.Stopped
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STOP" % (self.state))
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = 0.0
        self.rPower = 0.0
        self.lPower = 0.0
        self.logger.info('..stopped')

    # Part of the state machine. Invoked on receiving the TURN_RIGHT signal
    def turn_right(self, pwr):
        self.logger.info('Motor.turn_right(%d).in state %s..' % (pwr, self.state_names[self.state]))
        if self.state == self.Stopped:
            self.lf_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Right
        elif self.state == self.Turning_Right:
            self.lf_servo.ChangeDutyCycle(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Right
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            self.lr_servo.stop()
            self.lf_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Right
        elif self.state == self.Running_Fwd:
            self.rf_servo.stop()
            self.lf_servo.ChangeDutyCycle(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Right
        elif self.state == self.Running_Rev:
            self.lr_servo.stop()
            self.lf_servo.start(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Right
        elif self.state == self.Error_State:
            self.lf_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Right
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal TURN_RIGHT" % (self.state))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..turned_right. New state %s', (self.state_names[self.state]))

    # Part of the state machine. Invoked on receiving the TURN_LEFT signal
    def turn_left(self, pwr):
        self.logger.info('Motor.turn_left(%d)..in state %s' % (pwr, self.state_names[self.state]))
        if self.state == self.Stopped:
            self.rf_servo.start(pwr)
            self.lr_servo.start(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Turning_Left:
            self.rf_servo.ChangeDutyCycle(pwr)
            self.lr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Turning_Right:
            self.lf_servo.stop()
            self.rr_servo.stop()
            self.rf_servo.start(pwr)
            self.lr_servo.start(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Running_Fwd:
            self.lf_servo.stop()
            self.rf_servo.ChangeDutyCycle(pwr)
            self.lr_servo.start(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Running_Rev:
            self.rr_servo.stop()
            self.rf_servo.start(pwr)
            self.lr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Error_State:
            self.rf_servo.start(pwr)
            self.lr_servo.start(pwr)
            new_state = self.Turning_Left
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal TURN_LEFT" % (self.state))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..turned_left')

    # Part of the state machine. Invoked on receiving the STEER_RIGHT signal
    # pwr is the difference between left and right wheel power
    def steer_right(self, pwr):
        self.logger.info('Motor.steer_right(%d) at power %3.3f in state %s..' % (pwr, self.power, self.state_names[self.state]))
        hiPwr, loPwr = self.calc_power(self.power, pwr)
        if self.state == self.Stopped:
            new_state = self.Stopped
        elif self.state == self.Turning_Right:
            self.lr_servo.stop()
            self.rf_servo.stop()
            self.logger.error("ERROR: Motor state machine error. State Turning_Right. Received signal STEER_RIGHT")    
            new_state = self.Error_State
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            self.lr_servo.stop()
            self.logger.error("ERROR: Motor state machine error. State Turning_Left. Received signal STEER_RIGHT")    
            new_state = self.Error_State
        elif self.state == self.Running_Fwd:
            # print("steer_right. L: %3.3f R: %3.3f" % (hiPwr, loPwr))
            self.lf_servo.ChangeDutyCycle(hiPwr)
            self.rf_servo.ChangeDutyCycle(loPwr)
            self.lPower = hiPwr
            self.rPower = loPwr
            pwr = self.power
            new_state = self.Running_Fwd
        elif self.state == self.Running_Rev:
            self.lr_servo.ChangeDutyCycle(hiPwr)
            self.rr_servo.ChangeDutyCycle(loPwr)
            pwr = self.power
            self.lPwr = hiPwr
            self.rPwr = loPwr
            new_state = self.Running_Rev
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STEER_RIGHT" % (self.state))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..steered_right')

    # Part of the state machine. Invoked on receiving the STEER_LEFT signal
    # pwr is the difference between right and left wheel power
    def steer_left(self, pwr):
        self.logger.info('Motor.steer_left(%d) at power %3.3f in state %s' % (pwr, self.power, self.state_names[self.state]))
        hiPwr, loPwr = self.calc_power(self.power, pwr)
        if self.state == self.Stopped:
            new_state = self.Stopped
        elif self.state == self.Turning_Right:
            self.lr_servo.stop()
            self.rf_servo.stop()
            self.logger.error("ERROR: Motor state machine error. State Turning_Right. Received signal STEER_LEFT")    
            new_state = self.Error_State
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            self.lr_servo.stop()
            self.logger.error("ERROR: Motor state machine error. State Turning_Left. Received signal STEER_LEFT")    
            new_state = self.Error_State
        elif self.state == self.Running_Fwd:
            # print("steer_left. L: %3.3f R: %3.3f" % (loPwr, hiPwr))
            self.lf_servo.ChangeDutyCycle(loPwr)
            self.rf_servo.ChangeDutyCycle(hiPwr)
            pwr = self.power
            self.lPower = loPwr
            self.rPower = hiPwr
            new_state = self.Running_Fwd
        elif self.state == self.Running_Rev:
            self.lr_servo.ChangeDutyCycle(loPwr)
            self.rr_servo.ChangeDutyCycle(hiPwr)
            pwr = self.power
            self.lPower = loPwr
            self.rPower = hiPwr
            new_state = self.Running_Rev
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STEER_LEFT" % (self.state))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..steered_left')

    # Calculate the high- and low power settings when steering
    def calc_power(self, main_power, diff_pwr):
        half_pwr = diff_pwr / 2.0
        hiPower = main_power + half_pwr
        loPower = main_power - half_pwr
        if hiPower > 100.0:
            hiPower = 100.0
            loPower = main_power - diff_pwr
        elif loPower < 10.0:
            loPower = 10.0
            hiPower = 10.0 + diff_pwr
        if hiPower > 100:
            hiPower = 100
        if loPower < 10:
            loPower = 10
        # print("calc_power. main: %3.3f diff %3.3f hi %3.3f lo %3.3f" % (main_power, diff_pwr, hiPower, loPower))
        return hiPower, loPower

    # Part of the state machine. Invoked on receiving the RUN_FWD signal
    def run_fwd(self, pwr):
        self.logger.info('Motor.run_fwd(%d).in state %s..' % (pwr, self.state_names[self.state]))
        if self.state == self.Stopped:
            self.lf_servo.start(pwr)
            self.rf_servo.start(pwr)
            new_state = self.Running_Fwd
        elif self.state == self.Turning_Right:
            self.rr_servo.stop()
            self.rf_servo.start(pwr)
            self.lf_servo.ChangeDutyCycle(pwr)
            new_state = self.Running_Fwd
        elif self.state == self.Turning_Left:
            self.lr_servo.stop()
            self.lf_servo.start(pwr)
            self.rf_servo.ChangeDutyCycle(pwr)
            new_state = self.Running_Fwd
        elif self.state == self.Running_Fwd:
            self.lPower, self.rPower = self.calc_fwd_lr_power(pwr)
            self.lf_servo.ChangeDutyCycle(self.lPower)
            self.rf_servo.ChangeDutyCycle(self.rPower)
            new_state = self.Running_Fwd
        elif self.state == self.Running_Rev:
            self.lr_servo.stop()
            self.rr_servo.stop()
            self.lf_servo.start(pwr)
            self.rf_servo.start(pwr)
            new_state = self.Running_Fwd
        elif self.state == self.Error_State:
            self.lf_servo.start(pwr)
            self.rf_servo.start(pwr)
            new_state = self.Running_Fwd
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %s Received signal RUN_FWD" % (self.state_names[self.state]))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..ran_fwd. New state %d' % (self.state))

    def calc_fwd_lr_power(self, pwr):
        decelleration = self.power - pwr
        lr_diff = math.fabs(self.lPower - self.rPower)
        left = self.lPower - decelleration
        right = self.rPower - decelleration

        if left < 0:
            left = 0.0
        if right < 0:
            right = 0.0
        if left > 100:
            left = 100.0
            right = left - lr_diff
        elif right > 100:
            right = 100
            left = right - lr_diff
        return left, right
            

    # Part of the state machine. Invoked on receiving the RUN_REV signal
    def run_rev(self, pwr):
        self.logger.info('Motor.run_rev(%d).from state %s..' % (pwr, self.state_names[self.state]))
        if self.state == self.Stopped:
            self.lr_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Running_Rev
        elif self.state == self.Turning_Right:
            self.lf_servo.stop()
            self.lr_servo.start(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Running_Rev
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            self.rr_servo.start(pwr)
            self.lr_servo.ChangeDutyCycle(pwr)
            new_state = self.Running_Rev
        elif self.state == self.Running_Fwd:
            self.lf_servo.stop()
            self.rf_servo.stop()
            self.lr_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Running_Rev
        elif self.state == self.Running_Rev:
            self.lr_servo.ChangeDutyCycle(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Running_Rev
        elif self.state == self.Error_State:
            self.lr_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Running_Rev
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal RUN_REV" % (self.state))    
            self.force_stop()
            new_state = self.Stopped
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..ran_rev')



