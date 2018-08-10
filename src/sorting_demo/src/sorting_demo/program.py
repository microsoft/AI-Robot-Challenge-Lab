#!/usr/bin/env python
import rospkg

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
import sys

import demo_constants


def load_gazebo_models(table_pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('sawyer_sim_examples') + "/models/"
    # Load Table SDF
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open(model_path + "block/model.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))


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
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

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

    overhead_translation = [0.75*demo_constants.CUBE_EDGE_LENGHT,demo_constants.CUBE_EDGE_LENGHT/2.0,0.25*demo_constants.CUBE_EDGE_LENGHT]

    block_poses = list()

    original_pose_block = Pose(
        position=Point(x=0.45 , y=0.155, z=-0.129),
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

        if blocks is not None and len(blocks)>0:
            target_block = blocks[0][1] # access first item , pose field
            target_block.orientation = overhead_orientation

            target_block.position.x += overhead_translation[0]
            target_block.position.y += overhead_translation[1]
            target_block.position.z += overhead_translation[2]

            rospy.loginfo("blocks position:" + str(sorting_robot.environmentEstimation.get_blocks()) + "original\n" +str(original_pose_block))
            print("\nPicking...")
            sorting_robot.pick(target_block)
            print("\nPlacing...")
            #idx = (idx + 1) % len(block_poses)
            sorting_robot.place(target_block)
        else:
            sorting_robot.environmentEstimation.update()
            rospy.sleep(0.1)

    return 0


if __name__ == '__main__':
    sys.exit(main())
