import rospy
from gazebo_msgs.msg import LinkStates


class EnvironmentEstimation:
    def __init__(self):
        self.trays = []
        self.blocks = []

        # initial simulated implementation
        pub = rospy.Subscriber('/gazebo/link_states', LinkStates, self._links_callback, queue_size=10)

    def _links_callback(self, links):
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
        blocks = []
        trays = []

        for i, name in enumerate(links.name):
            if "block" in name:
                blocks.append(links.pose[i])
            elif "tray" in name:
                trays.append(links.pose[i])

        self.blocks = blocks
        self.trays = trays

    def get_blocks(self):
        """
        :return array of geometry_msgs.msg.Pose
        """
        return self.blocks

    def get_trays(self):
        """
        :return array of geometry_msgs.msg.Pose
        """
        return self.trays
