#!/usr/bin/python

import tracker
import motor_sw
import time
import sysprops


# s = sysprops.SysProps()
# s.set("collect_PID_data", False)
# s.store()
t = tracker.Tracker()
t.stop()
