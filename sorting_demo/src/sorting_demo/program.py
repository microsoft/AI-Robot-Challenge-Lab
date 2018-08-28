#!/usr/bin/python
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
    model_list, original_block_poses = gazebo_models.load_gazebo_models()
    # Remove models from the scene on shutdown

    rospy.on_shutdown(functools.partial(gazebo_models.delete_gazebo_models, model_list))

    task_planner = TaskPlanner()
    task_facade = task_planner.get_task_facade()

    task_facade.start()

    #rospy.sleep(40)
    #task_facade.put_all_contents_on_table()

    #task_facade.give_me_piece("GREEN")
    #task_facade.pause()

    #rospy.sleep(30)
    #task_facade.resume()

    #task_facade.stop("GREEN")


    task_planner.spin()

    # ask the robot to greet
    #task_facade.greet()



if __name__ == '__main__':
    sys.exit(main())
