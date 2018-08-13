#!/usr/bin/env python
import sys
import functools

import rospkg
import rospy

from geometry_msgs.msg import Pose, Point,Quaternion

import demo_constants
import gazebo_models

def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("sorting_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    model_list = gazebo_models.load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(functools.partial(gazebo_models.delete_gazebo_models, model_list))

    limb = 'right'
    hover_distance = 0.15  # meters
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4': -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}

    import sorting_robot
    from sorting_robot import SortingRobot
    sorting_robot = SortingRobot(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
        x=-0.00142460053167,
        y=0.999994209902,
        z=-0.00177030764765,
        w=0.00253311793936)

    overhead_translation = [0.75 * demo_constants.CUBE_EDGE_LENGHT, demo_constants.CUBE_EDGE_LENGHT / 2.0,
                            0.25 * demo_constants.CUBE_EDGE_LENGHT]

    block_poses = list()

    original_pose_block = Pose(
        position=Point(x=0.45, y=0.155, z=-0.129),
        orientation=overhead_orientation)

    block_poses.append(original_pose_block)

    """
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.45, y=0.155, z=-0.129),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.6, y=-0.1, z=-0.129),
        orientation=overhead_orientation))
    """

    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    sorting_robot.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        blocks = sorting_robot.environmentEstimation.get_blocks()
        rospy.loginfo("blocks: " + str(blocks))

        sorting_robot.environmentEstimation.update()

        if blocks is not None and len(blocks) > 0:
            target_block = blocks[0][1]  # access first item , pose field
            target_block.orientation = overhead_orientation

            target_block.position.x += overhead_translation[0]
            target_block.position.y += overhead_translation[1]
            target_block.position.z += overhead_translation[2]

            rospy.loginfo(
                "blocks position:" + str(sorting_robot.environmentEstimation.get_blocks()) + "original\n" + str(
                    original_pose_block))
            print("\nPicking...")
            sorting_robot.pick(target_block)
            print("\nPlacing...")
            # idx = (idx + 1) % len(block_poses)
            sorting_robot.place(target_block)
        else:
            rospy.sleep(0.1)

    return 0


if __name__ == '__main__':
    sys.exit(main())
