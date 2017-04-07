import RPi.GPIO as GPIO  
import time
import logging
import sysprops
import os

class Laser:
    """ IO interface to the IR laser weed killer
    """

    # GPIO pins
    LASER_IO = 26

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LASER_IO, GPIO.OUT)
        GPIO.output(self.LASER_IO, GPIO.HIGH)

    def off(self):
        GPIO.output(self.LASER_IO, GPIO.LOW)

    def on(self):
        GPIO.output(self.LASER_IO, GPIO.HIGH)
        

    
