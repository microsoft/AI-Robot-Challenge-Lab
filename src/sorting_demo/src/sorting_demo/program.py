#!/usr/bin/python
import sys
import os

# Assuming that pdvsd is located in the working folder
sys.path.append(os.getcwd())

import functools
import sys
import rospy

import demo_constants
import gazebo_models
from task_planner import TaskPlanner

def main():

    rospy.init_node("sorting_demo")
    rospy.logwarn("Hello world")

    if not demo_constants.is_real_robot():
        # Load Gazebo Models via Spawning Services
        # Note that the models reference is the /world frame
        # and the IK operates with respect to the /base frame
        model_list, original_block_poses = gazebo_models.load_gazebo_models()
        # Remove models from the scene on shutdown
        rospy.on_shutdown(functools.partial(gazebo_models.delete_gazebo_models, model_list))

    rospy.logwarn("Creating task planner")

    task_planner = TaskPlanner()

    task_planner.robot_say("Hello world").result()
    #rospy.spin()
    if not demo_constants.is_real_robot():
        task_planner.create_go_home_task(check_obstacles=False).result()

    task_facade = task_planner.get_task_facade()

<<<<<<< HEAD:src/sorting_demo/src/sorting_demo/program.py
    #task_facade.start()
=======

    if not demo_constants.is_real_robot():
        task_facade.start()
>>>>>>> aruco_on_real_robot:sorting_demo/src/sorting_demo/program.py

    task_facade.run_rest_server()

    # rospy.sleep(15)
    # task_facade.pick_block_by_color("BLUE")
    # task_facade.put_block_into_tray("BLUE", "1")

    # task_facade.put_all_contents_on_table()

    # task_facade.pause()

    # rospy.sleep(30)
    # task_facade.resume()

    # task_facade.stop("GREEN")

    rospy.logwarn("task planner spin")
    task_planner.spin()

    # ask the robot to greet
    # task_facade.greet()


if __name__ == '__main__':
    sys.exit(main())
