import laser
import time

l = laser.Laser()
l.off()
for i in range(20):
    time.sleep(0.5)
    l.on()
    time.sleep(0.5)
    l.off()
