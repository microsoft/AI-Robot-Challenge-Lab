import rospy
import gazebo_msgs.msg._LinkStates


class EnvironmentEstimation:
    def __init__(self):
        self.trays = []
        self.blocks = []

        # initial simulated implementation
        pub = rospy.Subscriber('/gazebo/link_states', gazebo_msgs.msg._LinkStates, self.links_callback, queue_size=10)

    def links_callback(self, links):
        """
        string[] name
        geometry_msgs/Pose[] pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        geometry_msgs/Twist[] twist
          geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z


        :param links: 
        :return: 
        """
        pass

    def get_blocks(self):
        return self.blocks
