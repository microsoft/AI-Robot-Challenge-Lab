#!/usr/bin/python

import sys
import os
import rospy
from threading import Thread
import time
import signal
import requests

#---------------------------------------------
def exit_properly_runtime_test():
    f = open("result.txt", "w")
    f.write("0")
    f.close()
    rospy.signal_shutdown("SUCCESS TEST")
    os.kill(os.getpid(), signal.SIGUSR1)
    sys.exit(0)

class ExitCommand(Exception):
    pass

def signal_handler(signal, frame):
    sys.exit(1)

#timeout countdown
def start_countdown(seconds):
    def timeout_countdown(seconds):
        time.sleep(seconds)
        rospy.logfatal("COUNTDOWN ERROR RUNTIME TEST")
        os.kill(os.getpid(), signal.SIGUSR1)
        sys.exit(1)
        
    t= Thread(target = lambda: timeout_countdown(seconds))
    t.start()
#---------------------------------------------
# launch countdown

signal.signal(signal.SIGUSR1, signal_handler)

import gazebo_msgs.msg
import intera_core_msgs.msg
rospy.init_node("test_node")
start_countdown(320)
rospy.logwarn("[Test Node] waiting from gazebo message, first time can be slow (gazebo has to download models)...")
rospy.wait_for_message("/gazebo/link_states", gazebo_msgs.msg.LinkStates,timeout=200)

# wait until box is picked up
def on_boxes_state_message(msg):
    #rospy.loginfo("received from test node: " + str(msg))

    str_msg = ""
    exit_flag = False
    for i, pose in enumerate(msg.pose):
        name = msg.name[i]    
        if "block" in name:
            str_msg +=  str(name) + ".y: "+ str(pose.position.y) +"\n"
            if pose.position.y >0.75:
                exit_flag = True
                rospy.logwarn("success, cube properly sent to tray. Ending!")
                break

    rospy.loginfo_throttle(1.0, "[Test Node] "+ str_msg)
    
    if exit_flag:
        exit_properly_runtime_test()

sub = rospy.Subscriber("/gazebo/link_states", gazebo_msgs.msg.LinkStates, on_boxes_state_message)

rospy.logwarn("[Test Node] waiting sorting demo...")
rospy.wait_for_message("/robot/limb/right/joint_command", intera_core_msgs.msg.JointCommand, timeout=60)

rospy.logwarn("[Test Node] Calling start sorting...")
rospy.sleep(10)
res = requests.get('http://localhost:5000/start')
rospy.logwarn(str(res))

rospy.spin()


