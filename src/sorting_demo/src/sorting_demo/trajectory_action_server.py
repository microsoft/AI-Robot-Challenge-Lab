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
Intera SDK Joint Trajectory Controller
    Unlike other robots running ROS, this is not a Motor Controller plugin,
    but a regular node using the SDK interface.
"""
import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
import intera_interface

from intera_joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

def start_server(limb, rate, mode, valid_limbs):
    rospy.loginfo("Initializing node... ")
    rospy.init_node("sdk_{0}_joint_trajectory_action_server{1}".format(
                        mode, "" if limb == 'all_limbs' else "_" + limb,))

    rospy.wait_for_service("/ExternalTools/right/PositionKinematicsNode/IKService")

    rospy.loginfo("Initializing joint trajectory action server...")
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    config_module = "intera_interface.cfg"
    if mode == 'velocity':
        config_name = ''.join([robot_name,"VelocityJointTrajectoryActionServerConfig"])
    elif mode == 'position':
        config_name = ''.join([robot_name,"PositionJointTrajectoryActionServerConfig"])
    else:
        config_name = ''.join([robot_name,"PositionFFJointTrajectoryActionServerConfig"])
    cfg = importlib.import_module('.'.join([config_module,config_name]))
    dyn_cfg_srv = Server(cfg, lambda config, level: config)
    jtas = []
    if limb == 'all_limbs':
        for current_limb in valid_limbs:
            jtas.append(JointTrajectoryActionServer(current_limb, dyn_cfg_srv,
                                                    rate, mode))
    else:
        jtas.append(JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode))


    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    rospy.loginfo("Joint Trajectory Action Server Running. Ctrl-c to quit")
    rospy.spin()


def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    # Add an option for starting a server for all valid limbs
    all_limbs = valid_limbs
    all_limbs.append("all_limbs")
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=all_limbs,
        help="joint trajectory action server limb"
    )
    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )
    parser.add_argument(
        "-m", "--mode", default='position_w_id',
        choices=['position_w_id', 'position', 'velocity'],
        help="control mode for trajectory execution"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    start_server(args.limb, args.rate, args.mode, valid_limbs)


if __name__ == "__main__":
    main()
