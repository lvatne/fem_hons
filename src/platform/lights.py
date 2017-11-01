import RPi.GPIO as GPIO  
import time

class Lights:
    HEADLIGHTS = 23

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.HEADLIGHTS, GPIO.OUT)
        GPIO.output(self.HEADLIGHTS, GPIO.LOW)

    def headlights(self, onoff):
        if onoff:
            GPIO.output(self.HEADLIGHTS, GPIO.HIGH)
        else:
            GPIO.output(self.HEADLIGHTS, GPIO.LOW)
            
        
