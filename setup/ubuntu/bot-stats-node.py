#!/usr/bin/env python

import rospy
from std_msgs.msg import String

print 'bot-stats-node starting'


def callback(data):
    print 'demostats callback'
    rospy.loginfo("I heard %s",data.data)
    
def listener():
    print 'demostats listener'
    rospy.init_node('Demo_Stats')
    rospy.Subscriber("/robot/state", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()