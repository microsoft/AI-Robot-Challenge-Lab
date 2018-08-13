#!/usr/bin/env python
import functools
import sys

import rospy
import gazebo_models
from task_planner import TaskPlanner


def main():

    rospy.init_node("sorting_demo")

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    model_list = gazebo_models.load_gazebo_models()
    # Remove models from the scene on shutdown

    rospy.on_shutdown(functools.partial(gazebo_models.delete_gazebo_models, model_list))

    task_planner = TaskPlanner()

    print("Running. Ctrl-c to quit")
    task_planner.run()


if __name__ == '__main__':
    sys.exit(main())
