#!/usr/bin/python
import sys

import rospkg
import rospy
import xacro

from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel


def spawn_urdf(name, description_xml, pose, reference_frame):
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf(name, description_xml, "/", pose, reference_frame)
    except rospy.ServiceException as e:
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
    except rospy.ServiceException as e:
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
    except rospy.ServiceException as e:
        print("Delete Model service call failed: {0}".format(e))
