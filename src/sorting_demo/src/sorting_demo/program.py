#!/usr/bin/env python
import sys
import functools

import rospkg
import rospy
import xacro

import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel

import demo_constants


def spawn_urdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def load_xacro_file(file_path, mappings):
    urdf_doc = xacro.process_file(file_path, mappings=mappings)
    urdf_xml = urdf_doc.toprettyxml(indent='  ', encoding='utf-8')
    urdf_xml = urdf_xml.replace('\n', '')
    return urdf_xml


def spawn_xacro_model(name, path, pose, reference_frame, mappings):
    description_xml = load_xacro_file(path, mappings)
    spawn_urdf(name, description_xml, pose, reference_frame)


def spawn_urdf_model(name, path, pose, reference_frame):
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')

    spawn_urdf(name, description_xml, pose, reference_frame)


def spawn_sdf_model(name, path, pose, reference_frame):
    # Load Model SDF
    description_xml = ''
    with open(path, "r") as model_file:
        description_xml = model_file.read().replace('\n', '')

    # Spawn Model SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def load_gazebo_models():
    model_list = []

    # Spawn Table
    table_name = "cafe_table"
    table_path = rospkg.RosPack().get_path('sawyer_sim_examples') + "/models/cafe_table/model.sdf"
    table_pose = Pose(position=Point(x=0.75, y=0.0, z=0.0))
    table_reference_frame = "world"
    spawn_sdf_model(table_name, table_path, table_pose, table_reference_frame)
    model_list.append(table_name)

    # Spawn blocks
    block_path = rospkg.RosPack().get_path('sorting_demo') + "/models/block/block.urdf.xacro"
    block_reference_frame = "world"

    block_poses = [
        Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)),
        Pose(position=Point(x=0.60, y=0.1265, z=0.7725)),
        Pose(position=Point(x=0.4225, y=-0.1, z=0.7725))]
    block_mappings = [
        {"material": "Gazebo/Green"},
        {"material": "Gazebo/Orange"},
        {"material": "Gazebo/SkyBlue"}]

    for (i, pose, mappings) in zip(range(len(block_poses)), block_poses, block_mappings):
        name = "block{}".format(i)
        spawn_xacro_model(name, block_path, pose, block_reference_frame, mappings)
        model_list.append(name)

    return model_list


def delete_gazebo_models(model_list):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        for model in model_list:
            resp_delete = delete_model(model)
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
    model_list = load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(functools.partial(delete_gazebo_models, model_list))

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
