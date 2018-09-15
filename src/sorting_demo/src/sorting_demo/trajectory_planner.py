#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg

from geometry_msgs.msg import Point, Pose
import demo_constants


class TrajectoryPlanner:
    def __init__(self):
        """
        
        """

        self.ceilheight = 0.75
        rospy.sleep(0.4)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(0.4)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        rospy.sleep(0.1)

        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory)
        self.planning_scene_diff_publisher = rospy.Publisher("planning_scene", moveit_msgs.msg.PlanningScene,
                                                             queue_size=1)

        rospy.sleep(0.1)

        self.set_default_planner()

        print "============ Reference frame: %s" % self.group.get_planning_frame()

        print "============ Reference frame: %s" % self.group.get_end_effector_link()
        print self.robot.get_group_names()
        print self.robot.get_current_state()
        self.enable_collision_table1 = True
        self.enable_orientation_constraint = False
        self.set_default_tables_z()
        self.registered_blocks = []

        self.tableshape = (0.913, 0.913, 0.01)
        #self.tableshape = (1.2, 1.2, 0.01)

        rospy.sleep(0.2)

    def register_box(self, pose):
        self.registered_blocks.append(pose)
        rospy.sleep(0.1)

    def set_default_tables_z(self):
        self.table1_z = demo_constants.TABLE_HEIGHT + 0.02
        self.table2_z = demo_constants.TABLE_HEIGHT + 0.05

    def set_default_planner(self):
        # self.group.set_planner_id("OMPL")
        # self.group.set_planner_id("RRTConnectk
        # ConfigDefault")
        # self.group.set_planner_id("LBKPIECEkConfigDefault")
        self.group.set_planner_id("ESTkConfigDefault")
        # commit
        rospy.sleep(0.2)

    def pick(self, block):
        target_pose = block.grasp_pose

        self.group.set_num_planning_attempts(300)
        self.create_environment_obstacles()
        block_index = self.update_block(block)
        self.group.set_support_surface_name("table1")

        grasp = moveit_msgs.msg.Grasp()
        grasp.grasp_pose.header.frame_id = "world"
        grasp.grasp_pose.header.stamp = rospy.Time.now()
        grasp.grasp_pose.pose = target_pose
        grasp.grasp_pose.pose.position.z+=0.1

        grasp.pre_grasp_approach.direction.vector.z = -1.0
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.15

        # opened gripper
        grasp.pre_grasp_posture.joint_names = ["right_gripper_l_finger_joint", "right_gripper_r_finger_joint"]
        trajpoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajpoint.positions = [0.04, 0.04]
        trajpoint.time_from_start = rospy.Duration(0.5)
        grasp.pre_grasp_posture.points = [trajpoint]

        grasp.post_grasp_retreat.direction.vector.z = 1.0
        grasp.post_grasp_retreat.min_distance = 0.095
        grasp.post_grasp_retreat.desired_distance = 0.15

        # closed gripper
        grasp.grasp_posture.joint_names = ["right_gripper_l_finger_joint", "right_gripper_r_finger_joint"]
        trajpoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajpoint.positions = [0.0, 0.0]
        trajpoint.time_from_start = rospy.Duration(0.5)
        grasp.grasp_posture.points = [trajpoint]

        graps = [grasp]
        rospy.logwarn("blocks that can be picked: " + str(self.registered_blocks))

        return self.group.pick("block" + str(block_index), graps)

    def place(self,block):

        self.create_environment_obstacles()

        place_location = moveit_msgs.msg.PlaceLocation()
        place_location.place_pose.header.frame_id = "world"
        place_location.place_pose.pose = block.tray_place_pose

        place_location.pre_place_approach.direction.header.frame_id="world"
        place_location.pre_place_approach.direction.vector.z = -1.0
        place_location.pre_place_approach.min_distance = 0.2
        place_location.pre_place_approach.desired_distance = 0.115

        place_location.post_place_retreat.direction.header.frame_id = "world"
        place_location.post_place_retreat.direction.vector.z = 1.0
        place_location.post_place_retreat.min_distance = 0.1
        place_location.post_place_retreat.desired_distance = 0.2

        #open
        place_location.post_place_posture.joint_names = ["right_gripper_l_finger_joint", "right_gripper_r_finger_joint"]
        trajpoint = trajectory_msgs.msg.JointTrajectoryPoint()
        trajpoint.positions = [0.04, 0.04]
        trajpoint.time_from_start = rospy.Duration(0.5)
        place_location.post_place_posture.points = [trajpoint]
        self.group.set_support_surface_name("table2")
        rospy.sleep(1.0)
        return self.group.place("block" + str(self.registered_blocks.index(block)), place_location)


    def update_block(self, block):
        rospy.logwarn("updating block collision info")
        if not block in self.registered_blocks:
            raise Exception("block does not exist")

        rospy.logwarn("current blocks: "+ str(self.registered_blocks))
        block_pose = geometry_msgs.msg.PoseStamped()
        block_pose.pose = block.grasp_pose
        block_pose.header.stamp = rospy.Time.now()
        block_pose.header.frame_id = self.robot.get_planning_frame()
        block_index = self.registered_blocks.index(block)
        self.scene.add_box("block" + str(block_index), block_pose, size=(0.04, 0.04, 0.04))
        rospy.sleep(0.5)
        return block_index


    def register_table1(self):
        table1pose = geometry_msgs.msg.PoseStamped()
        table1pose.pose = Pose(position=Point(x=0.75, y=0.0, z=self.table1_z))
        table1pose.pose.orientation.w = 1.0
        table1pose.header.stamp = rospy.Time.now()
        table1pose.header.frame_id = "world"
        self.scene.add_box("table1", table1pose, size=self.tableshape)

    def update_table2(self):
        table2pose = geometry_msgs.msg.PoseStamped()
        table2pose.pose = Pose(position=Point(x=0.0, y=1.0, z=self.table2_z))
        table2pose.pose.orientation.w = 1.0
        table2pose.header.stamp = rospy.Time.now()
        table2pose.header.frame_id = "world"
        self.scene.add_box("table2", table2pose, size=self.tableshape)


        edgezsize = 0.3
        edgex = geometry_msgs.msg.PoseStamped()
        edgex.pose = Pose(position=Point(x=0.0, y=1.0 - self.tableshape[1]/2.0, z=self.table2_z + edgezsize*0.5))
        edgex.pose.orientation.w = 1.0
        edgex.header.stamp = rospy.Time.now()
        edgex.header.frame_id = "world"

        newshape = (self.tableshape[0], self.tableshape[1]*0.01, self.tableshape[2]+edgezsize)
        self.scene.add_box("table2_edgex", edgex, size=newshape)


        edgex2 = geometry_msgs.msg.PoseStamped()
        edgex2.pose = Pose(position=Point(x=0.0, y=1.0 + self.tableshape[1]/2.0, z=self.table2_z  + edgezsize*0.5))
        edgex2.pose.orientation.w = 1.0
        edgex2.header.stamp = rospy.Time.now()
        edgex2.header.frame_id = "world"

        newshape = (self.tableshape[0], self.tableshape[1]*0.01, self.tableshape[2]+edgezsize)
        self.scene.add_box("table2_edgex2", edgex2, size=newshape)

        edgey = geometry_msgs.msg.PoseStamped()
        edgey.pose = Pose(position=Point(x=self.tableshape[0] / 2.0, y=1.0 , z=self.table2_z  + edgezsize*0.5))
        edgey.pose.orientation.w = 1.0
        edgey.header.stamp = rospy.Time.now()
        edgey.header.frame_id = "world"

        newshape = (self.tableshape[0]*0.01, self.tableshape[1] , self.tableshape[2] + edgezsize)
        self.scene.add_box("table2_edgey", edgey, size=newshape)

        edgey2 = geometry_msgs.msg.PoseStamped()
        edgey2.pose = Pose(position=Point(x= - self.tableshape[0] / 2.0, y=1.0 , z=self.table2_z  + edgezsize*0.5))
        edgey2.pose.orientation.w = 1.0
        edgey2.header.stamp = rospy.Time.now()
        edgey2.header.frame_id = "world"

        newshape = (self.tableshape[0]*0.01, self.tableshape[1] , self.tableshape[2] + edgezsize  + edgezsize*0.5)
        self.scene.add_box("table2_edgey2", edgey2, size=newshape)

        splits = 3
        for i in xrange(1,splits):
            edgeyi = geometry_msgs.msg.PoseStamped()
            edgeyi.pose = Pose(position=Point(x= i*self.tableshape[0] / float(splits) - self.tableshape[0]*0.5, y=1.0, z=self.table2_z + edgezsize * 0.5))
            edgeyi.pose.orientation.w = 1.0
            edgeyi.header.stamp = rospy.Time.now()
            edgeyi.header.frame_id = "world"

            newshape = (self.tableshape[0] * 0.01, self.tableshape[1], self.tableshape[2] + edgezsize + edgezsize * 0.5)
            self.scene.add_box("table2_edgey_"+str(i), edgeyi, size=newshape)

    def create_environment_obstacles(self):
        """
        :return: 
        """
        ceilshape = (2.0, 2.0, 0.02)
        backwall = (0.01, 0.25, 1.4)
        largexwall = (0.01, 2.0, 1.4)
        ywall = (2.0, 0.01, 1.4)


        ceil1pose = geometry_msgs.msg.PoseStamped()
        ceil1pose.pose = Pose(position=Point(x=0.0, y=0.0, z=self.ceilheight))
        ceil1pose.pose.orientation.w = 1.0
        ceil1pose.header.stamp = rospy.Time.now()
        ceil1pose.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("ceil1", ceil1pose, size=ceilshape)

        xwallpose = geometry_msgs.msg.PoseStamped()
        xwallpose.pose = Pose(position=Point(x=-0.35, y=0.0, z=0.0))
        xwallpose.pose.orientation.w = 1.0
        xwallpose.header.stamp = rospy.Time.now()
        xwallpose.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("backwall", xwallpose, size=backwall)

        postxwallpose = geometry_msgs.msg.PoseStamped()
        postxwallpose.pose = Pose(position=Point(x=-0.9, y=0.0, z=0.0))
        postxwallpose.pose.orientation.w = 1.0
        postxwallpose.header.stamp = rospy.Time.now()
        postxwallpose.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("postbackwall", postxwallpose, size=largexwall)

        frontwall = geometry_msgs.msg.PoseStamped()
        frontwall.pose = Pose(position=Point(x=1.1, y=0.0, z=0.0))
        frontwall.pose.orientation.w = 1.0
        frontwall.header.stamp = rospy.Time.now()
        frontwall.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("frontwall", frontwall, size=largexwall)

        traywall = geometry_msgs.msg.PoseStamped()
        traywall.pose = Pose(position=Point(x=-0.0, y=1.5, z=0.0))
        traywall.pose.orientation.w = 1.0
        traywall.header.stamp = rospy.Time.now()
        traywall.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("traywall", traywall, size=ywall)

        frontsidewall = geometry_msgs.msg.PoseStamped()
        frontsidewall.pose = Pose(position=Point(x=-0.0, y=-1., z=0.0))
        frontsidewall.pose.orientation.w = 1.0
        frontsidewall.header.stamp = rospy.Time.now()
        frontsidewall.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("frontsidewall", frontsidewall, size=ywall)

        rospy.sleep(0.1)

    def path_constraints(self, target_pose):
        """
        
        :return: 
        """
        constraints = moveit_msgs.msg.Constraints()

        orientationconstraint = moveit_msgs.msg.OrientationConstraint()
        # orientationconstraint.orientation.w=1
        orientationconstraint.orientation.x = target_pose.orientation.x
        orientationconstraint.orientation.y = target_pose.orientation.y
        orientationconstraint.orientation.z = target_pose.orientation.z
        orientationconstraint.orientation.w = target_pose.orientation.w

        orientationconstraint.absolute_x_axis_tolerance = 0.1
        orientationconstraint.absolute_y_axis_tolerance = 0.1
        orientationconstraint.absolute_z_axis_tolerance = 0.1
        orientationconstraint.weight = 1.0

        orientationconstraint.link_name = "right_l6"
        orientationconstraint.header.frame_id = "world"
        orientationconstraint.header.stamp = rospy.Time.now()

        if self.enable_orientation_constraint:
            constraints.orientation_constraints.append(orientationconstraint)

        """
        jcm = moveit_msgs.msg.JointConstraint()
        jcm.joint_name = "right_j0"
        jcm.position = 0
        jcm.tolerance_above = 0.5
        jcm.tolerance_below = 0.5
        jcm.weight = 0.5
        constraints.joint_constraints.append(jcm)

        jcm = moveit_msgs.msg.JointConstraint()
        jcm.joint_name = "right_j1"
        jcm.position = 0
        jcm.tolerance_above = 0.5
        jcm.tolerance_below = 0.5
        jcm.weight = 0.5
        constraints.joint_constraints.append(jcm)

        rospy.sleep(0.1)
        """

        return constraints

    def clear_parameters(self):
        """
        
        :return: 
        """
        self.group.clear_pose_targets()
        self.group.clear_path_constraints()
        rospy.sleep(0.5)

    def publish_planning_scene(self):
        """
        collision_objects = self.scene.get_objects()
        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects = collision_objects
        planning_scene.is_diff = True

        print planning_scene
        self.planning_scene_diff_publisher.publish(planning_scene)
        """

    def move_to_joint_target(self, joints, currentpose=None, attempts=10):
        """
        
        :param joints: float array
        :return: 
        """

        self.clear_parameters()

        #self.create_environment_obstacles()

        # self.group.set_goal_joint_tolerance	(0.2)
        try:
            self.group.set_joint_value_target(joints)
        except Exception as ex:
            pass

        self.group.set_num_planning_attempts(attempts)

        rospy.sleep(0.3)

        if currentpose is None:
            self.group.set_start_state_to_current_state()
            currentpose = self.group.get_current_pose().pose
        else:
            # self.group.set_start_state(currentpose)

            # rospy.logwarn(self.robot.get_current_state())
            # currentpose = self.group.get_current_pose(self.group.get_end_effector_link()).pose
            self.group.set_start_state_to_current_state()
            constraints = self.path_constraints(currentpose)
            self.group.set_path_constraints(constraints)

        rospy.sleep(0.1)

        plan1 = self.group.plan()
        rospy.sleep(0.1)
        rospy.logwarn("PLAN: " + str(plan1))

        self.publish_planning_scene()

        # rospy.logwarn("PLAAAAN!!! "+ str(plan1))

        if plan1.joint_trajectory.header.frame_id != "":
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()

            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan1)

            self.display_trajectory_publisher.publish(display_trajectory)

            success = self.group.go(wait=True)
            rospy.logwarn("result of the motion: " + str(success))
            return success
        else:
            return False

    def move_to_cartesian_target(self, pose_target, attempts=100):
        """
        
        :param pose_target: geometry_msgs/Pose
        :return: 
        """
        self.clear_parameters()

        self.create_environment_obstacles()

        self.group.set_pose_target(pose_target)
        self.group.set_num_planning_attempts(attempts)

        rospy.sleep(0.3)

        constraints = self.path_constraints(self.group.get_current_pose().pose)
        self.group.set_path_constraints(constraints)

        plan1 = self.group.plan()
        rospy.logwarn(plan1)
        rospy.sleep(0.1)

        self.publish_planning_scene()

        # rospy.logwarn("PLAAAAN!!! "+ str(plan1))

        if plan1.joint_trajectory.header.frame_id != "":
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()

            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan1)

            self.display_trajectory_publisher.publish(display_trajectory)

            success = self.group.go(wait=True)
            rospy.logwarn("result of the motion: " + str(success))
            return success
        else:
            return False
