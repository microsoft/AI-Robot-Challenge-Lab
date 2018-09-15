import math
import rospy


def force_joint_limits(jntspos):
    joint_limits = [(-3.0503, 3.0503), (-3.8095, 2.2736), (-3.0426, 3.0426), (-3.0439, 3.0439)]

    rospy.logwarn(jntspos)
    for i in xrange(len(joint_limits)):
        lower = joint_limits[i][0]
        upper = joint_limits[i][1]
        if jntspos[i] < lower and (jntspos[i] + 2.0 * math.pi) < upper:
            jntspos[i] = jntspos[i] + 2 * math.pi

        if jntspos[i] > upper and (jntspos[i] - 2.0 * math.pi) > lower:
            jntspos[i] = jntspos[i] - 2 * math.pi

    return jntspos