import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import demo_constants
from sawyer_robot_facade import SawyerRobotFacade


class TaskPlanner:
    def __init__(self):
        limb = 'right'
        hover_distance = 0.15  # meters
        self.sawyer_robot = SawyerRobotFacade(limb, hover_distance)

    def create_go_home_task(self):
        # Starting Joint angles for right arm
        starting_joint_angles = {'right_j0': -0.041662954890248294,
                                 'right_j1': -1.0258291091425074,
                                 'right_j2': 0.0293680414401436,
                                 'right_j3': 2.17518162913313,
                                 'right_j4': -0.06703022873354225,
                                 'right_j5': 0.3968371433926965,
                                 'right_j6': 1.7659649178699421}

        self.sawyer_robot.move_to_start(starting_joint_angles)

    def create_pick_task(self):
        # An orientation for gripper fingers to be overhead and parallel to the obj
        overhead_orientation = Quaternion(
            x=-0.00142460053167,
            y=0.999994209902,
            z=-0.00177030764765,
            w=0.00253311793936)

        original_pose_block = Pose(
            position=Point(x=0.45, y=0.155, z=-0.129),
            orientation=overhead_orientation)

        block_poses = list()

        block_poses.append(original_pose_block)

        overhead_translation = [0.75 * demo_constants.CUBE_EDGE_LENGHT, demo_constants.CUBE_EDGE_LENGHT / 2.0,
                                0.25 * demo_constants.CUBE_EDGE_LENGHT]

        blocks = self.sawyer_robot.environmentEstimation.get_blocks()
        rospy.loginfo("blocks: " + str(blocks))

        if blocks is not None and len(blocks) > 0:
            target_block = blocks[0][1]  # access first item , pose field
            target_block.orientation = overhead_orientation

            target_block.position.x += overhead_translation[0]
            target_block.position.y += overhead_translation[1]
            target_block.position.z += overhead_translation[2]

            rospy.loginfo(
                "blocks position:" + str(self.sawyer_robot.environmentEstimation.get_blocks()) + "original\n" + str(
                    original_pose_block))

            print("\nPicking task...")
            self.sawyer_robot.pick(target_block)

    def create_place_task(self, target_pose):
        print("\nPlacing task...")
        # idx = (idx + 1) % len(block_poses)
        self.sawyer_robot.place(target_pose)

    def run(self):
        """
        main planner loop that executes all the behaviors and open paralelly other tasks with their own loop
        :return: 
        """

        # Move to the desired starting angles
        self.create_go_home_task()

        while not rospy.is_shutdown():
            self.sawyer_robot.environmentEstimation.update()

            if self.create_pick_task():
                self.create_place_task()
            else:
                rospy.sleep(0.1)

        return 0
