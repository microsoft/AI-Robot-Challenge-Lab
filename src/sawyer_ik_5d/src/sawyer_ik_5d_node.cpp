#include <urdf/model.h>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <moveit_msgs/GetPositionIK.h>

using namespace KDL;
using namespace Eigen;
using namespace ros;
using namespace std;

#define SAWYER_JOINT_COUNT 6

ChainIkSolverPos_LMA *ikSolverPos;
ChainFkSolverPos_recursive *fkSolverPos;
Tree tree;
Chain chain;
Eigen::Matrix<double,6/*pose params*/,1> L;
vector<string> joint_names;
JntArray q_current(SAWYER_JOINT_COUNT);

ros::ServiceServer ikservice;
bool initialized= false;

void getParameters()
{
    NodeHandle nh;
	////////////////////////////////////////////////////////////
	// Get robot description
	////////////////////////////////////////////////////////////
	string description;
	if(!nh.getParam("/robot_description", description))
	{
		ROS_ERROR("Parameter not set: /robot_description");
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Create robot model
	////////////////////////////////////////////////////////////
	urdf::Model model;
	if(!model.initString(description))
	{
		ROS_ERROR("Could not initialize robot model");
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Get tip link name and verify it exists in the model
	////////////////////////////////////////////////////////////
	string tip_link_name;
	if(!nh.getParam("tip_link_name", tip_link_name))
	{
        tip_link_name = "right_hand_camera_optical";
		ROS_WARN("Parameter not set: %s/tip_link_name, default value -> %s", nh.getNamespace().c_str(), tip_link_name.c_str());
		//exit(-1);
	}
	if(model.links_.find(tip_link_name) == model.links_.end())
	{
		ROS_ERROR("The link specified in %s/tip_link_name does not exist", nh.getNamespace().c_str());
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Get revolute joints
	////////////////////////////////////////////////////////////
	vector<boost::shared_ptr<urdf::Joint> > model_joints;

	for(boost::shared_ptr<const urdf::Link> current_link = model.getLink(tip_link_name); current_link->parent_joint != NULL; current_link = current_link->getParent())
	{
		if(current_link->parent_joint->type == urdf::Joint::REVOLUTE)
		{
			model_joints.insert(model_joints.begin(), current_link->parent_joint);
		}
	}

	if(model_joints.size() != SAWYER_JOINT_COUNT)
	{
		ROS_ERROR("The robot model must have %d revolute joints, found %d", SAWYER_JOINT_COUNT, (int)model_joints.size());
		exit(-1);
	}

	////////////////////////////////////////////////////////////
	// Configure servo limits
	////////////////////////////////////////////////////////////
	// for(int joint_i = 0; joint_i < SAWYER_JOINT_COUNT; joint_i++)
	// {
	// 	float lowerLimit = model_joints[joint_i]->limits->lower;
	// 	float upperLimit = model_joints[joint_i]->limits->upper;
	// }

	////////////////////////////////////////////////////////////
	// Store joint names
	////////////////////////////////////////////////////////////
	for(int joint_i = 0; joint_i < SAWYER_JOINT_COUNT; joint_i++)
	{
		joint_names.push_back(model_joints[joint_i]->name);
	}

	////////////////////////////////////////////////////////////
	// Create KDL tree
	////////////////////////////////////////////////////////////
	if(!kdl_parser::treeFromString(description, tree))
	{
		ROS_ERROR("Failed to construct kdl tree");
		exit(-1);
	}

	if(!tree.getChain(model.getRoot()->name, tip_link_name, chain))
	{
		ROS_ERROR("Failed to make chain from tree");
		exit(-1);
	}
}

void initialize_ik()
{
    getParameters();
    
    ROS_INFO("creating solvers");
    // Create KDL solvers
	L(0) = 1; L(1) = 1; L(2) = 1; L(3) = 1; L(4) = 1; L(5) = 0 ;
	ikSolverPos = new KDL::ChainIkSolverPos_LMA(chain, L);
	fkSolverPos = new KDL::ChainFkSolverPos_recursive(chain);
}

bool ik(moveit_msgs::GetPositionIK::Request& request, moveit_msgs::GetPositionIK::Response& response)
{
    ROS_INFO("ik request");
    if(!initialized)
    {
        ROS_INFO("initializing");
        initialize_ik();
        ROS_INFO("initialized");
    }

    ROS_INFO("reading inputs");
    for(int i=0;i<SAWYER_JOINT_COUNT;i++)
    {
        q_current(i) = request.ik_request.robot_state.joint_state.position[i];
    }

    const geometry_msgs::Point& p = request.ik_request.pose_stamped.pose.position;
    const geometry_msgs::Quaternion& q = request.ik_request.pose_stamped.pose.orientation;

    Frame target_frame (Rotation::Quaternion(q.x,q.y,q.z,q.w),Vector(p.x,p.y,p.z));

    ROS_INFO("computing ik, target %lf %lf %lf", target_frame.p[0],target_frame.p[1],target_frame.p[2]);
	  JntArray q_res;
    int result = ikSolverPos->CartToJnt(q_current, target_frame,q_res);
    ROS_INFO("IK RESULT: %d", result);
    //0 if successful, -1 the gradient of $ E $ towards the joints is to small, -2 if joint position increments are to small, -3 if number of iterations is exceeded.

    sensor_msgs::JointState& respjoints= response.solution.joint_state;

    respjoints.name= request.ik_request.robot_state.joint_state.name;

    for(int i=0;i<SAWYER_JOINT_COUNT;i++)
        respjoints.position.push_back(q_res(i));

    return true;
}

int main(int argc, char** argv)
{
    // Init ROS node
	ros::init(argc, argv, "sawyer_ik_5d");

    // Get node handle
	NodeHandle nh("~");

    ikservice = nh.advertiseService("ik", ik);

    ros::spin();
}

/*
moveit_msgs/PositionIKRequest ik_request
  string group_name
  moveit_msgs/RobotState robot_state
    sensor_msgs/JointState joint_state
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string[] name
      float64[] position
      float64[] velocity
      float64[] effort
    sensor_msgs/MultiDOFJointState multi_dof_joint_state
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string[] joint_names
      geometry_msgs/Transform[] transforms
        geometry_msgs/Vector3 translation
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion rotation
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
      geometry_msgs/Wrench[] wrench
        geometry_msgs/Vector3 force
          float64 x
          float64 y
          float64 z
        geometry_msgs/Vector3 torque
          float64 x
          float64 y
          float64 z
    moveit_msgs/AttachedCollisionObject[] attached_collision_objects
      string link_name
      moveit_msgs/CollisionObject object
        byte ADD=0
        byte REMOVE=1
        byte APPEND=2
        byte MOVE=3
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string id
        object_recognition_msgs/ObjectType type
          string key
          string db
        shape_msgs/SolidPrimitive[] primitives
          uint8 BOX=1
          uint8 SPHERE=2
          uint8 CYLINDER=3
          uint8 CONE=4
          uint8 BOX_X=0
          uint8 BOX_Y=1
          uint8 BOX_Z=2
          uint8 SPHERE_RADIUS=0
          uint8 CYLINDER_HEIGHT=0
          uint8 CYLINDER_RADIUS=1
          uint8 CONE_HEIGHT=0
          uint8 CONE_RADIUS=1
          uint8 type
          float64[] dimensions
        geometry_msgs/Pose[] primitive_poses
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        shape_msgs/Mesh[] meshes
          shape_msgs/MeshTriangle[] triangles
            uint32[3] vertex_indices
          geometry_msgs/Point[] vertices
            float64 x
            float64 y
            float64 z
        geometry_msgs/Pose[] mesh_poses
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        shape_msgs/Plane[] planes
          float64[4] coef
        geometry_msgs/Pose[] plane_poses
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        byte operation
      string[] touch_links
      trajectory_msgs/JointTrajectory detach_posture
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string[] joint_names
        trajectory_msgs/JointTrajectoryPoint[] points
          float64[] positions
          float64[] velocities
          float64[] accelerations
          float64[] effort
          duration time_from_start
      float64 weight
    bool is_diff
  moveit_msgs/Constraints constraints
    string name
    moveit_msgs/JointConstraint[] joint_constraints
      string joint_name
      float64 position
      float64 tolerance_above
      float64 tolerance_below
      float64 weight
    moveit_msgs/PositionConstraint[] position_constraints
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string link_name
      geometry_msgs/Vector3 target_point_offset
        float64 x
        float64 y
        float64 z
      moveit_msgs/BoundingVolume constraint_region
        shape_msgs/SolidPrimitive[] primitives
          uint8 BOX=1
          uint8 SPHERE=2
          uint8 CYLINDER=3
          uint8 CONE=4
          uint8 BOX_X=0
          uint8 BOX_Y=1
          uint8 BOX_Z=2
          uint8 SPHERE_RADIUS=0
          uint8 CYLINDER_HEIGHT=0
          uint8 CYLINDER_RADIUS=1
          uint8 CONE_HEIGHT=0
          uint8 CONE_RADIUS=1
          uint8 type
          float64[] dimensions
        geometry_msgs/Pose[] primitive_poses
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        shape_msgs/Mesh[] meshes
          shape_msgs/MeshTriangle[] triangles
            uint32[3] vertex_indices
          geometry_msgs/Point[] vertices
            float64 x
            float64 y
            float64 z
        geometry_msgs/Pose[] mesh_poses
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
      float64 weight
    moveit_msgs/OrientationConstraint[] orientation_constraints
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
      string link_name
      float64 absolute_x_axis_tolerance
      float64 absolute_y_axis_tolerance
      float64 absolute_z_axis_tolerance
      float64 weight
    moveit_msgs/VisibilityConstraint[] visibility_constraints
      uint8 SENSOR_Z=0
      uint8 SENSOR_Y=1
      uint8 SENSOR_X=2
      float64 target_radius
      geometry_msgs/PoseStamped target_pose
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
      int32 cone_sides
      geometry_msgs/PoseStamped sensor_pose
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
      float64 max_view_angle
      float64 max_range_angle
      uint8 sensor_view_direction
      float64 weight
  bool avoid_collisions
  string ik_link_name
  geometry_msgs/PoseStamped pose_stamped
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  string[] ik_link_names
  geometry_msgs/PoseStamped[] pose_stamped_vector
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  duration timeout
  int32 attempts
---
moveit_msgs/RobotState solution
  sensor_msgs/JointState joint_state
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
  sensor_msgs/MultiDOFJointState multi_dof_joint_state
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] joint_names
    geometry_msgs/Transform[] transforms
      geometry_msgs/Vector3 translation
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion rotation
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
    geometry_msgs/Wrench[] wrench
      geometry_msgs/Vector3 force
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 torque
        float64 x
        float64 y
        float64 z
  moveit_msgs/AttachedCollisionObject[] attached_collision_objects
    string link_name
    moveit_msgs/CollisionObject object
      byte ADD=0
      byte REMOVE=1
      byte APPEND=2
      byte MOVE=3
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string id
      object_recognition_msgs/ObjectType type
        string key
        string db
      shape_msgs/SolidPrimitive[] primitives
        uint8 BOX=1
        uint8 SPHERE=2
        uint8 CYLINDER=3
        uint8 CONE=4
        uint8 BOX_X=0
        uint8 BOX_Y=1
        uint8 BOX_Z=2
        uint8 SPHERE_RADIUS=0
        uint8 CYLINDER_HEIGHT=0
        uint8 CYLINDER_RADIUS=1
        uint8 CONE_HEIGHT=0
        uint8 CONE_RADIUS=1
        uint8 type
        float64[] dimensions
      geometry_msgs/Pose[] primitive_poses
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      shape_msgs/Mesh[] meshes
        shape_msgs/MeshTriangle[] triangles
          uint32[3] vertex_indices
        geometry_msgs/Point[] vertices
          float64 x
          float64 y
          float64 z
      geometry_msgs/Pose[] mesh_poses
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      shape_msgs/Plane[] planes
        float64[4] coef
      geometry_msgs/Pose[] plane_poses
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      byte operation
    string[] touch_links
    trajectory_msgs/JointTrajectory detach_posture
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      string[] joint_names
      trajectory_msgs/JointTrajectoryPoint[] points
        float64[] positions
        float64[] velocities
        float64[] accelerations
        float64[] effort
        duration time_from_start
    float64 weight
  bool is_diff
moveit_msgs/MoveItErrorCodes error_code
  int32 SUCCESS=1
  int32 FAILURE=99999
  int32 PLANNING_FAILED=-1
  int32 INVALID_MOTION_PLAN=-2
  int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
  int32 CONTROL_FAILED=-4
  int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
  int32 TIMED_OUT=-6
  int32 PREEMPTED=-7
  int32 START_STATE_IN_COLLISION=-10
  int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-11
  int32 GOAL_IN_COLLISION=-12
  int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-13
  int32 GOAL_CONSTRAINTS_VIOLATED=-14
  int32 INVALID_GROUP_NAME=-15
  int32 INVALID_GOAL_CONSTRAINTS=-16
  int32 INVALID_ROBOT_STATE=-17
  int32 INVALID_LINK_NAME=-18
  int32 INVALID_OBJECT_NAME=-19
  int32 FRAME_TRANSFORM_FAILURE=-21
  int32 COLLISION_CHECKING_UNAVAILABLE=-22
  int32 ROBOT_STATE_STALE=-23
  int32 SENSOR_INFO_STALE=-24
  int32 NO_IK_SOLUTION=-31
  int32 val

*/