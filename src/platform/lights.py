import RPi.GPIO as GPIO  
import time

class Lights:
    """ Controlling lights (illuminating lights, not indicators) on the robot.
        The 'headlights' means the light bar used for photography
    """
    
    HEADLIGHTS = 23  # GPIO pin for the light bar (runs to  a 12V driver circuit)

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.HEADLIGHTS, GPIO.OUT)
        GPIO.output(self.HEADLIGHTS, GPIO.LOW)

    def headlights(self, onoff):
        if onoff:
            GPIO.output(self.HEADLIGHTS, GPIO.HIGH)
        else:
            GPIO.output(self.HEADLIGHTS, GPIO.LOW)
            
        
