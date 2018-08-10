import tf
import tf.transformations
import numpy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion


def inverse_compose(basehomopose, homopose):
    """
    :param basehomopose: 4x4 homogeneous transform matrix
    :param homopose:  4x4 homogeneous transform matrix
    :return:
    """
    return numpy.matmul(numpy.linalg.inv(basehomopose), homopose)


def get_homo_matrix_from_pose_msg(pose, tag=""):
    basetrans = tf.transformations.translation_matrix((pose.position.x,
                                                       pose.position.y,
                                                       pose.position.z))

    baserot = tf.transformations.quaternion_matrix((pose.orientation.x,
                                                    pose.orientation.y,
                                                    pose.orientation.z,
                                                    pose.orientation.w))

    # rospy.loginfo(tag + " basetrans: " + str(basetrans))
    # rospy.loginfo(tag +" baserot: " + str(baserot))

    combined = numpy.matmul(basetrans, baserot)

    # rospy.loginfo(tag + " combined: " + str(combined))
    trans = tf.transformations.translation_from_matrix(combined)
    quat = tf.transformations.quaternion_from_matrix(combined)

    # rospy.loginfo(tag + " back basetrans: " + str(trans))
    # rospy.loginfo(tag +" back baserot: " + str(quat))

    return combined


def homotransform_to_pose_msg(homotransform):
    trans = tf.transformations.translation_from_matrix(homotransform)
    quat = tf.transformations.quaternion_from_matrix(homotransform)
    return Pose(
        position=Point(x=trans[0], y=trans[1], z=trans[2]),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
