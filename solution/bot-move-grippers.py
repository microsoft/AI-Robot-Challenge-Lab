#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Gripper Example: keyboard
"""
import argparse

import rospy
import time

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION

def move_gripper(limb, action):
    # initialize interfaces
    print("Getting robot state...")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state()
    gripper = None
    original_deadzone = None
    
    def clean_shutdown():
        if gripper and original_deadzone:
            gripper.set_dead_zone(original_deadzone)
        print("Exiting example.")
    
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except (ValueError, OSError) as e:
        rospy.logerr("Could not detect an electric gripper attached to the robot.")
        clean_shutdown()
        return
    rospy.on_shutdown(clean_shutdown)

    original_deadzone = gripper.get_dead_zone()
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    rospy.loginfo("Enabling robot...")
    rs.enable()
    print("Controlling grippers.")
    if (action == "open"):
        gripper.open()
        rospy.sleep(1.0)
        print("Opened grippers")
    elif (action == "close"):
        gripper.close()
        rospy.sleep(1.0)
        print("Closed grippers")
    
    # force shutdown call if caught by key handler
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Gripper Example: send a command to control the grippers.

    Run this example to command various gripper movements while
    adjusting gripper parameters, including calibration, and velocity:
    Uses the intera_interface.Gripper class and the
    helper function, intera_external_devices.getch.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    limb = valid_limbs[0]
    print("Using limb: {}.".format(limb))
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-a", "--action", dest="action", default="open",
        choices=["open", "close"],
        help="Action to perform with the gripper. Options: close or open"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_gripper_keyboard")

    move_gripper(limb, args.action)


if __name__ == '__main__':
    main()
