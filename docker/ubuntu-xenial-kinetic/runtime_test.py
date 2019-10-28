#!/usr/bin/python

import sys
import os
import rospy
from threading import Thread
import time

#---------------------------------------------
def exit_properly_runtime_test():
    f = open("result.txt", "w")
    f.write("0")
    f.close()
    sys.exit(0)

#timeout countdown
def start_countdown(seconds):
    def timeout_countdown(secibds):
        time.sleep(seconds)
        rospy.logfatal("COUNTDOWN ERROR RUNTIME TEST")
        sys.exit(1)

    t= Thread(target = lambda: timeout_countdown(10))
    t.start()
#---------------------------------------------
# launch countdown

start_countdown(4)
time.sleep(3)


exit_properly_runtime_test()

