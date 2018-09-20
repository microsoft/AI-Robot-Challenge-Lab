#! /usr/bin/env python

import geometry_msgs
import rospy

from sorting_demo.trajectory_planner import TrajectoryPlanner

if __name__ == "__main__":
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    planner = TrajectoryPlanner()

    pose_target = geometry_msgs.msg.Pose()

    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.7
    pose_target.position.y = 0.0
    pose_target.position.z = 0.5

    #planner.move_to_cartesian_target(pose_target)

    planner.move_to_joint_target([0.0,0.0,0.0,0.0,0.0,0.0,0.0])