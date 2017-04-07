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

    # Signals
    STOP = 10
    TURN_RIGHT = 11
    TURN_LEFT = 12
    STEER_RIGHT = 13
    STEER_LEFT = 14
    RUN_FWD = 15
    RUN_REV = 16

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

        self.lf_servo = GPIO.PWM(self.LEFT_FORWARD, 30)
        self.lr_servo = GPIO.PWM(self.LEFT_REVERSE, 30)
        self.rf_servo = GPIO.PWM(self.RIGHT_FORWARD, 30)
        self.rr_servo = GPIO.PWM(self.RIGHT_REVERSE, 30)

        self.power = 0.0
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
        self.logger.setLevel(logging.INFO)

    # Entry point for the motor state machine
    def signal(self, sign, pwr):
        self.prev_signal = self.curr_signal
        self.curr_signal = sign
        
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

    
    # Part of the state machine. Invoked on receiving the STOP signal
    def stop(self):
        self.logger.info('stopping...')
        if self.state == self.Stopped:
            new_state = self.Stopped
        elif self.state == self.Turning_Right:
            self.lf_servo.stop()
            # GPIO.output(self.LEFT_FORWARD,GPIO.LOW)
            self.rr_servo.stop()
            # GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)
            new_state = self.Stopped
        elif self.state == self.Turning_Left:
            self.rf_servo.stop()
            # GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
            self.lr_servo.stop()
            # GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
            new_state = self.Stopped
        elif self.state == self.Running_Fwd:
            self.lf_servo.stop()
            # GPIO.output(self.LEFT_FORWARD,GPIO.LOW)
            self.rf_servo.stop()
            # GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)
            new_state = self.Stopped
        elif self.state == self.Running_Rev:
            self.lr_servo.stop()
            # GPIO.output(self.LEFT_REVERSE,GPIO.LOW)
            self.rr_servo.stop()
            # GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)
            new_state = self.Stopped
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STOP" % (self.state))
            
        self.prev_state = self.state
        self.state = new_state
        self.power = 0.0
        self.logger.info('..stopped')

    # Part of the state machine. Invoked on receiving the TURN_RIGHT signal
    def turn_right(self, pwr):
        self.logger.info('Motor.turn_right(%d).in state %d..' % (pwr, self.state))
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
            self.rr_servo.start(pwr)
            self.lf_servo.ChangeDutyCycle(pwr)
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
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..turned_right. New state %d', (self.state))

    # Part of the state machine. Invoked on receiving the TURN_LEFT signal
    def turn_left(self, pwr):
        self.logger.info('Motor.turn_left(%d)...' % (pwr))
        if self.state == self.Stopped:
            self.lf_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Turning_Left:
            self.lf_servo.ChangeDutyCycle(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Turning_Right:
            self.lf_servo.stop()
            self.rr_servo.stop()
            self.rf_servo.start(pwr)
            self.lr_servo.start(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Running_Fwd:
            self.rf_servo.stop()
            self.rr_servo.start(pwr)
            self.lf_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Running_Rev:
            self.lr_servo.stop()
            self.lf_servo.start(pwr)
            self.rr_servo.ChangeDutyCycle(pwr)
            new_state = self.Turning_Left
        elif self.state == self.Error_State:
            self.lf_servo.start(pwr)
            self.rr_servo.start(pwr)
            new_state = self.Turning_Left
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal TURN_LEFT" % (self.state))    
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..turned_left')

    # Part of the state machine. Invoked on receiving the STEER_RIGHT signal
    def steer_right(self, pwr):
        self.logger.info('Motor.steer_right(%d)...' % (pwr))
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
            self.lf_servo.ChangeDutyCycle(self.power + pwr)
            self.rf_servo.ChangeDutyCycle(self.power - pwr)
            pwr = self.power
            new_state = self.Running_Fwd
        elif self.state == self.Running_Rev:
            self.lr_servo.ChangeDutyCycle(self.power + pwr)
            self.rr_servo.ChangeDutyCycle(self.power - pwr)
            pwr = self.power
            new_state = self.Running_Rev
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STEER_RIGHT" % (self.state))    
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..steered_right')

    # Part of the state machine. Invoked on receiving the STEER_LEFT signal
    def steer_left(self, pwr):
        self.logger.info('Motor.steer_left(%d)...' % (pwr))
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
            self.lf_servo.ChangeDutyCycle(self.power - pwr)
            self.rf_servo.ChangeDutyCycle(self.power + pwr)
            pwr = self.power
            new_state = self.Running_Fwd
        elif self.state == self.Running_Rev:
            self.lr_servo.ChangeDutyCycle(self.power - pwr)
            self.rr_servo.ChangeDutyCycle(self.power + pwr)
            pwr = self.power
            new_state = self.Running_Rev
        elif self.state == self.Error_State:
            new_state = self.Stopped
        else:
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal STEER_LEFT" % (self.state))    
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..steered_left')


    # Part of the state machine. Invoked on receiving the RUN_FWD signal
    def run_fwd(self, pwr):
        self.logger.info('Motor.run_fwd(%d).in state %d..' % (pwr, self.state))
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
            self.lf_servo.ChangeDutyCycle(pwr)
            self.rf_servo.ChangeDutyCycle(pwr)
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
            self.logger.error("ERROR: Motor state machine error. Unknown state %d Received signal RUN_FWD" % (self.state))    
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..ran_fwd. New state %d' % (self.state))

    # Part of the state machine. Invoked on receiving the RUN_REV signal
    def run_rev(self, pwr):
        self.logger.info('Motor.run_rev(%d)...' % (pwr))
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
        self.prev_state = self.state
        self.state = new_state
        self.power = pwr
        self.logger.info('..ran_rev')



    def left_fwd(self, speed) :
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        self.lf_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.lf_servo.ChangeDutyCycle(speed);

    def left_rev(self, speed) :
        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)
        self.lr_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.lr_servo.ChangeDutyCycle(speed)
        
    def right_fwd(self, speed) :
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)  
        self.rf_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.rf_servo.ChangeDutyCycle(speed)
        
        
    def right_rev(self, speed) :
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        self.rr_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.rr_servo.ChangeDutyCycle(speed)

    def turn_calc_sec(self, dega) :
        sec = dega / self.TURN_FACTOR
        return sec

    def neutral_calc(self, dega) :
        sec = dega / self.NEUTRAL_TURN_FACTOR
        return sec


     # High level methods

    def set_power(self, power):
        """ Positive power means go forward. Negative power means go backward
        """
        self.power = power
        self.logger.info("Motor.set_power(%f)" % (power))
        if power > 0:
            self.forward(power)
        elif power < 0:
            self.reverse(math.fabs(power))
        else:
            self.stop()

    def steer(self, adj):
        """ React to the output of the PID controller.
        The range of input is -100 to +100
        When the gyro gives negative values, the robot is turning to the right (going forward)
        This gives a negative Controller Output (CO) value
        When receiving a positive steering value, the robot should turn towards  right (going forward)
        When receiving a negative steering value, the robot should turn towards left (going forward)

        Make sure that power does not fall between -50 and 50, as the motor will stop drawing.
        """
        
        # if self.dbg_cnt % 25 == 0:
        #     print("Steer(%3.3f)" % (adj))
        self.dbg_cnt = self.dbg_cnt + 1
        rSpeed = self.power
        lSpeed = self.power
        mid = adj / 2        
        if adj > 0.5 or adj < -0.5: # Skip miniscule adjustments
            if self.power > 10: # We are going forward 
                if adj >= 0: # We want to turn to the right
                    lSpeed = self.power + mid  
                    if lSpeed > 100:
                        lSpeed = 100
                    if lSpeed < 50:
                        lSpeed = 50
                    rSpeed = lSpeed - adj
                    if rSpeed < 0:
                        rSpeed = 0
                else: # We want to turn to the left
                    rSpeed = self.power - mid # Remember, mid is negative here
                    if rSpeed > 100:
                        rSpeed = 100
                    if rSpeed < 50:
                        rSpeed = 50
                    lSpeed = rSpeed + adj
                    if lSpeed < 0:
                        lSpeed = 0
                self.lf_servo.ChangeDutyCycle(lSpeed)
                self.rf_servo.ChangeDutyCycle(rSpeed)
            elif self.power > -9 and self.power < 9: # standing still - sorta
                if adj >= 0: # Will turn right
                    self.lf_servo.ChangeDutyCycle(adj)
                    self.rr_servo.ChangeDutyCycle(adj)
                else: # Will turn left
                    adj = math.fabs(adj)
                    self.rf_servo.ChangeDutyCycle(adj)
                    self.lr_servo.ChangeDutyCycle(adj)
                    
            else:
                self.lr_servo.ChangeDutyCycle(self.power - mid)
                self.rr_servo.ChangeDutyCycle(self.power + mid)
        
            
    def start_rotation(self, angle):
        self.stop()
        if angle > 2:
            self.lf_servo.start(100)
            self.rr_servo.start(100)
        elif angle < -2:
            self.rf_servo.start(100)
            self.lr_servo.start(100)
            
                    
            
    def forward(self, speed) :
        self.power = speed
        self.logger.info("forward(%f)" % (speed))
        GPIO.output(self.LEFT_REVERSE,GPIO.LOW)  
        GPIO.output(self.RIGHT_REVERSE,GPIO.LOW)  
        self.lf_servo.start(100)
        self.rf_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.lf_servo.ChangeDutyCycle(speed);
        self.rf_servo.ChangeDutyCycle(speed)
        self.logger.info("forward() proceeding")

    def reverse(self, speed) :
        self.power = 0 - speed
        self.logger.info("reverse(%f)" % (speed))
        GPIO.output(self.LEFT_FORWARD,GPIO.LOW)
        GPIO.output(self.RIGHT_FORWARD,GPIO.LOW)  
        self.lr_servo.start(100)
        self.rr_servo.start(100)
        time.sleep(self.ROLL_UP_TIME)
        self.lr_servo.ChangeDutyCycle(speed)
        self.rr_servo.ChangeDutyCycle(speed)        
        self.logger.info("reverse() proceeding")


    def right_turn(self, dega) :
        self.logger.info("right_turn({})".format(dega))
        sec = self.turn_calc_sec(dega)
        self.stop()
        self.left_fwd(80)
        time.sleep(sec)
        self.stop()
        self.logger.info("right_turn() done")

    def left_turn(self, dega) :
        self.logger.info("left_turn({})".format(dega))
        sec = self.turn_calc_sec(dega)
        self.stop()
        self.right_fwd(80)
        time.sleep(sec)
        self.stop()
        self.logger.info("left_turn() done")

    def right_neutral_turn(self, dega) :
        self.logger.info("right_neutral_turn({})".format(dega))
        sec = self.neutral_calc(dega)
        self.stop()
        self.left_fwd(100)
        self.right_rev(100)
        time.sleep(sec)
        self.stop()
        self.logger.info("right_neutral_turn() done")

    def left_neutral_turn(self, dega) :
        self.logger.info("left_neutral_turn({})".format(dega))
        sec = self.neutral_calc(dega)
        self.stop()
        self.right_fwd(80)
        self.left_rev(80)
        time.sleep(sec)
        self.stop()
        self.logger.info("left_neutral_turn() done")
        
        
        
        
