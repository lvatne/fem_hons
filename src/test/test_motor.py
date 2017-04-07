
import motor_sw
import time

m = motor_sw.Motor_sw()
m.force_stop()

# m.signal(m.RUN_FWD, 100)
# time.sleep(0.4)
# m.signal(m.RUN_FWD, 70)
# time.sleep(0.6)
# m.signal(m.STEER_LEFT,20)
# time.sleep(1)
# m.signal(m.STEER_RIGHT, 20)
# time.sleep(1)

# m.signal(m.RUN_REV, 90)
# time.sleep(1)
# m.signal(m.RUN_REV, 70)
# time.sleep(0.5)
# m.signal(m.STEER_RIGHT, 20)
# time.sleep(1)
# m.signal(m.STEER_LEFT,20)
# time.sleep(1)
         

# m.signal(m.TURN_RIGHT, 80)
# time.sleep(2)
# m.signal(m.TURN_LEFT, 80)
# time.sleep(2)

# m.signal(m.STEER_LEFT,20)
# time.sleep(1)
# m.signal(m.STEER_RIGHT, 20)
# time.sleep(1)
m.signal(m.STOP, 0)

         
