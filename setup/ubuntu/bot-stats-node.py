#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from intera_core_msgs.msg import AssemblyState
import sys
import json

print 'bot-stats-node starting'


def callback(data):
    print 'demostats callback'
    #rospy.loginfo("I heard %s",data.data)
    print json.dumps(data)
    
def main():
    print 'demostats listener'
    rospy.init_node('Demo_Stats')
    rospy.Subscriber("/robot/state", AssemblyState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())

