#!/usr/bin/env python

import sys
import rospy
import intera_interface

from std_msgs.msg import String
from intera_interface import CHECK_VERSION


def main():
    rospy.init_node('Demo_Stats')
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    state = rs.state()
    print state
    rospy.signal_shutdown("Demo_Stats finished.")


if __name__ == '__main__':
    sys.exit(main())

