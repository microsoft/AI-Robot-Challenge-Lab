#!/usr/bin/python
import functools
import sys
import rospy

from task_planner import TaskPlanner
import demo_constants
import gazebo_models

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

    rospy.logwarn("Hello world")


    task_planner.create_go_home_task(check_obstacles=False).result()

    task_facade = task_planner.get_task_facade()

    #task_facade.start()

    task_facade.run_rest_server()

    #rospy.sleep(15)
    #task_facade.pick_block_by_color("BLUE")
    #task_facade.put_block_into_tray("BLUE", "1")

    #task_facade.put_all_contents_on_table()

    #task_facade.pause()

    #rospy.sleep(30)
    #task_facade.resume()

    #task_facade.stop("GREEN")

    rospy.logwarn("task planner spin")
    task_planner.spin()

    # ask the robot to greet
    #task_facade.greet()



if __name__ == '__main__':
    sys.exit(main())
