import RPi.GPIO as GPIO  
import time

class Lights:
    HEADLIGHTS = 4

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.HEADLIGHTS, GPIO.OUT)

    def headlights(self, onoff):
        if onoff:
            GPIO.output(self.HEADLIGHTS, GPIO.HIGH)
        else:
            GPIO.output(self.HEADLIGHTS, GPIO.LOW)
            
        
