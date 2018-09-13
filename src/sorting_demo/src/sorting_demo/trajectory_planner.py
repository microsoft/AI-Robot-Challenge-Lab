#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from geometry_msgs.msg import Point, Pose
import demo_constants


class TrajectoryPlanner:
    def __init__(self):
        """
        
        """

        self.ceilheight = 0.75
        rospy.sleep(3)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.sleep(2)

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

        rospy.sleep(0.2)

    def set_default_planner(self):
        # self.group.set_planner_id("OMPL")
        self.group.set_planner_id("RRTConnectkConfigDefault")
        # self.group.set_planner_id("LBKPIECEkConfigDefault")
        self.group.set_planner_id("ESTkConfigDefault")
        # commit
        rospy.sleep(0.2)

    def create_environment_obstacles(self):
        """
        :return: 
        """

        tableshape = (0.913, 0.913, 0.1)
        ceilshape = (2.0, 2.0, 0.02)
        backwall = (0.01, 0.25, 1.4)
        largexwall = (0.01, 2.0, 1.4)
        ywall = (2.0, 0.01, 1.4)

        table1pose = geometry_msgs.msg.PoseStamped()
        table1pose.pose = Pose(position=Point(x=0.75, y=0.0, z=0.0))
        table1pose.pose.orientation.w = 1.0
        table1pose.pose.position.z = demo_constants.TABLE_HEIGHT
        table1pose.header.stamp = rospy.Time.now()
        table1pose.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("table1", table1pose, size=tableshape)

        table2pose = geometry_msgs.msg.PoseStamped()
        table2pose.pose = Pose(position=Point(x=0.0, y=1.0, z=0.0))
        table2pose.pose.orientation.w = 1.0
        table2pose.pose.position.z = demo_constants.TABLE_HEIGHT
        table2pose.header.stamp = rospy.Time.now()
        table2pose.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box("table2", table2pose, size=tableshape)

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

    def path_constraints(self):
        """
        
        :return: 
        """
        constraints = moveit_msgs.msg.Constraints()

        orientationconstraint = moveit_msgs.msg.OrientationConstraint()
        # orientationconstraint.orientation.w=1
        orientationconstraint.orientation.x = -0.011325648436031916
        orientationconstraint.orientation.y = 0.9998115142702567
        orientationconstraint.orientation.z = -0.006101035043221461
        orientationconstraint.orientation.w = 0.014541079448283218

        orientationconstraint.absolute_x_axis_tolerance = 1.5
        orientationconstraint.absolute_y_axis_tolerance = 1.5
        orientationconstraint.absolute_z_axis_tolerance = 1.5
        orientationconstraint.weight = 1.0

        orientationconstraint.link_name = "right_l6"
        orientationconstraint.header.frame_id = "world"
        orientationconstraint.header.stamp = rospy.Time.now()

        # constraints.orientation_constraints.append(orientationconstraint)

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

    def move_to_joint_target(self, joints, attempts=300):
        """
        
        :param joints: float array
        :return: 
        """
        self.clear_parameters()

        # self.create_environment_obstacles()

        # self.group.set_goal_joint_tolerance	(0.2)
        try:
            self.group.set_joint_value_target(joints)
        except Exception as ex:
            pass

        self.group.set_num_planning_attempts(attempts)

        rospy.sleep(0.3)

        constraints = self.path_constraints()
        self.group.set_path_constraints(constraints)
        rospy.sleep(0.1)

        plan1 = self.group.plan()
        rospy.sleep(0.1)

        self.publish_planning_scene()
        rospy.logwarn(plan1)

        # rospy.logwarn("PLAAAAN!!! "+ str(plan1))

        if plan1.joint_trajectory.header.frame_id != "":
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()

            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan1)

            self.display_trajectory_publisher.publish(display_trajectory)

            self.group.go(wait=True)
            return True
        else:
            return False

    """
    def original_servo_to_pose_loop(self, target_pose, time=4.0, steps=400.0):
        r = rospy.Rate(1 / (time / steps))  # Defaults to 100Hz command rate
        current_pose = self._limb.endpoint_pose()
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - target_pose.position.x) / steps
        ik_delta.position.y = (current_pose['position'].y - target_pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - target_pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - target_pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - target_pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - target_pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - target_pose.orientation.w) / steps
        for d in range(int(steps), -1, -1):
            if rospy.is_shutdown():
                return
            ik_step = Pose()
            ik_step.position.x = d * ik_delta.position.x + target_pose.position.x
            ik_step.position.y = d * ik_delta.position.y + target_pose.position.y
            ik_step.position.z = d * ik_delta.position.z + target_pose.position.z
            ik_step.orientation.x = d * ik_delta.orientation.x + target_pose.orientation.x
            ik_step.orientation.y = d * ik_delta.orientation.y + target_pose.orientation.y
            ik_step.orientation.z = d * ik_delta.orientation.z + target_pose.orientation.z
            ik_step.orientation.w = d * ik_delta.orientation.w + target_pose.orientation.w
            joint_angles = self._limb.ik_request(ik_step, self._tip_name)
            if joint_angles:
                self._limb.set_joint_positions(joint_angles)
            else:
                rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

            r.sleep()
        r.sleep()
    """

    def move_to_cartesian_target(self, pose_target):
        """
        
        :param pose_target: geometry_msgs/Pose
        :return: 
        """
        self.clear_parameters()

        # self.create_environment_obstacles()

        self.group.set_pose_target(pose_target)
        self.group.set_num_planning_attempts(10)

        constraints = self.path_constraints()
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

            self.group.go(wait=True)
            return True
        else:
            return False
