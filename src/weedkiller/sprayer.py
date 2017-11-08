import RPi.GPIO as GPIO  
import time
import logging
import sysprops
import os

class Sprayer:
    """ IO interface to the IR laser weed killer
    """

    # GPIO pins
    SPRAYER_IO = 19

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SPRAYER_IO, GPIO.OUT)
        GPIO.output(self.SPRAYER_IO, GPIO.LOW)
        self.off()

    def off(self):
        GPIO.output(self.SPRAYER_IO, GPIO.LOW)

    def on(self):
        GPIO.output(self.SPRAYER_IO, GPIO.HIGH)

    def squirt(self):
        self.on()
        time.sleep(0.1)
        self.off()
        
        

    
