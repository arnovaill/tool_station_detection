#include <ros/package.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

#include <eigen_conversions/eigen_msg.h>
#include <eigen_helper_functions/eigen_helper_functions.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf_conversions/tf_eigen.h>

// #include <flexbotics_tool_exchanger_manager_msgs/SetMovementParameters.h>
// #include <flexbotics_tool_exchanger_manager_msgs/SetUseSimParameter.h>
// #include <flexbotics_tool_exchanger_manager_msgs/LoadToolExchangerInfo.h>
// #include <flexbotics_tool_exchanger_manager_msgs/GrabTool.h>
// #include <flexbotics_tool_exchanger_manager_msgs/ReleaseTool.h>
// #include <tool_exchanger_info_manager/tool_exchanger_info_manager.h>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <flexbotics_manipulator_manager_msgs/MoveTraj.h>
#include <flexbotics_manipulation_planner_msgs/PlanMoveTcp.h> 
#include <tool_station_detection_msgs/ToolStationDetection.h>
// #include <flexbotics_manipulation_planner_msgs/PlanMoveLin.h>
// #include <flexbotics_manipulation_planner_msgs/PlanMoveJoint.h> 


ros::NodeHandle *nh_;

// // Movement related parameters 
// double velocity_ratio_;
// double translation_velocity_;
// double rotation_velocity_;
// double timeout_; 
// int max_count_; // for functions "extra" (trying multiple times to execute a function)

// Program variables
std::string package_path_;
std::string marker_frame_;
std::string output_frame_;
std::string camera_frame_;


// marker variables
ar_track_alvar_msgs::AlvarMarkersConstPtr p_markers_;
geometry_msgs::Pose marker_pose_msg_;
std::vector<Eigen::Affine3d> marker_pose_;
Eigen::Affine3d marker_pose_mean_;
std::vector<std::deque<Eigen::Affine3d>> marker_pose_history_;
int n_marker_;
int size_marker_memory_;
std::vector<int> marker_id_;

// Services
ros::ServiceServer tool_station_detection_service_;
// // ros::ServiceServer set_use_sim_parameter_service_;

// Subscribers
ros::Subscriber ar_marker_pose_;

// Clients
ros::ServiceClient move_traj_client_;
// ros::ServiceClient plan_move_lin_client_;
ros::ServiceClient plan_move_tcp_client_;
// // ros::ServiceClient plan_move_joint_client_;


void printPose(Eigen::Affine3d pose, std::string name)
{
  std::vector<double> xyz, rpy;
  eigen_helper_functions::extractXyzRpy(pose, xyz, rpy);
  tf::Quaternion q = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
  ROS_WARN("%s :  XYZ %3.3f %3.3f %3.3f - RPY %3.3f %3.3f %3.3f - QUAT %f %f %f %f ",
            name.c_str(), xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2], q[0], q[1], q[2], q[3]);
}

bool averagePose(std::deque<Eigen::Affine3d> poses, Eigen::Affine3d& mean_pose)
{
  std::vector<double> xyz, rpy;
  std::vector<double> xyz_mean = {0,0,0};
  std::vector<double> rpy_mean = {0,0,0};
  int n_pose = poses.size();

  for(int i = 0; i < n_pose; i++)
  {
    eigen_helper_functions::extractXyzRpy(poses[i], xyz, rpy);
    xyz_mean[0] += xyz[0] ;
    xyz_mean[1] += xyz[1] ;
    xyz_mean[2] += xyz[2] ;
    rpy_mean[0] += rpy[0] ;
    rpy_mean[1] += rpy[1] ;
    rpy_mean[2] += rpy[2] ;
  }

  for (int i = 0; i < 3; i++) 
  {
    xyz_mean[i] = xyz_mean[i] / n_pose;
    rpy_mean[i] = rpy_mean[i] / n_pose;
  }
  
  eigen_helper_functions::createHomogeneousMatrixXyzRpy( xyz_mean, rpy_mean , mean_pose);
  
  return true;
}

// Function from tf_eigen.cpp
void transformTFToEigen(const tf::Transform &t, Eigen::Affine3d & e)
{
  for(int i=0; i<3; i++)
  {
    e.matrix()(i,3) = t.getOrigin()[i];
    for(int j=0; j<3; j++)
    {
      e.matrix()(i,j) = t.getBasis()[i][j];        
    }      
  }
  // Fill in identity in last row
  for (int col = 0 ; col < 3; col ++) e.matrix()(3, col) = 0;

  e.matrix()(3,3) = 1;
}

void poseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& msg)
{

  n_marker_ = msg->markers.size();
  // ROS_INFO_THROTTLE(2,"Number of marker detected: %d ", n_marker_);
  Eigen::Affine3d marker_pose;

  if (n_marker_ >= 1)
  {
    // reset marker pose to have good number of element
    marker_pose_.clear();
    marker_id_.clear();
    for (int i = 0; i < n_marker_ ; i++)
    {
        marker_pose_msg_ = msg->markers[i].pose.pose;
        tf::poseMsgToEigen(marker_pose_msg_, marker_pose);

        marker_id_.push_back(msg->markers[i].id);
        marker_pose_.push_back(marker_pose);

        // ROS_INFO("marker_id: %d", marker_id_[i]);
        // printPose(marker_pose_[i],"x");


        // marker_pose_history_[i].push_front(marker_pose_);

        // if (marker_pose_history_[i].size() > size_marker_memory_)
        // {
        //   // keep only the last n marker poses
        //   marker_pose_history_[i].resize(size_marker_memory_);
        //   // average
        //   averagePose(marker_pose_history_, marker_pose_mean_[i]);
        // }
      

    }
  }
    
}


bool initVariables() 
{
  // ROS
  package_path_ = ros::package::getPath("tool_station_detection");
  size_marker_memory_ = 15;
  // Service Clients
  // plan_move_lin_client_ = nh_->serviceClient<flexbotics_manipulation_planner_msgs::PlanMoveLin>("/flexbotics_manipulation_planner/plan_move_lin");
  // plan_move_joint_client_ = nh_->serviceClient<flexbotics_manipulation_planner_msgs::PlanMoveJoint>("/flexbotics_manipulation_planner/plan_move_joint");
  plan_move_tcp_client_ = nh_->serviceClient<flexbotics_manipulation_planner_msgs::PlanMoveTcp>("/flexbotics_manipulation_planner/plan_move_tcp");
  move_traj_client_ = nh_->serviceClient<flexbotics_manipulator_manager_msgs::MoveTraj>("flexbotics_manipulator_manager_node/move_traj"); 

  // Subscribers
  ar_marker_pose_ = nh_->subscribe("ar_pose_marker", 100, poseMarkerCallback);

  if (nh_->hasParam("/tool_station_detection/camera_frame"))
  {
    nh_->getParam("/tool_station_detection/camera_frame", camera_frame_);
  }
  if (nh_->hasParam("/tool_station_detection/output_frame"))
  {
    nh_->getParam("/tool_station_detection/output_frame", output_frame_);
  }

  return true;
}


bool MoveTraj(std::string group_name, trajectory_msgs::JointTrajectory trajectory , double timeout )
{ 

  flexbotics_manipulator_manager_msgs::MoveTraj move_traj_service;

  // Assign service arguments
  move_traj_service.request.group_name = group_name;
  move_traj_service.request.trajectory = trajectory;
  move_traj_service.request.timeout = timeout;

  if (move_traj_client_.call(move_traj_service))
  {
    ROS_INFO("Success to call service 'flexbotics_manipulator_manager_node/move_traj' ");
    ROS_INFO("    move_traj output variables:");
    ROS_INFO("        success = %d", move_traj_service.response.success);
    ROS_INFO("        error_id = %d", move_traj_service.response.error_id);
    ROS_INFO("        message = %s", move_traj_service.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service 'flexbotics_manipulator_manager_node/move_traj' ");
    return false;
  }

  return true;
}

bool PlanMoveTcp(std::string group_name, bool use_actual_pose, std::vector<double> initial_joint_position, 
                Eigen::Affine3d target_tcp_pose,  Eigen::Affine3d initial_tcp_pose, double velocity_ratio,
                trajectory_msgs::JointTrajectory& trajectory)
{
   flexbotics_manipulation_planner_msgs::PlanMoveTcp plan_move_tcp_service;

 
  geometry_msgs::Pose target_tcp_pose_msg, initial_tcp_pose_msg;
  //Conversion from Eigen to geometry msg
  tf::poseEigenToMsg(target_tcp_pose,target_tcp_pose_msg);
  tf::poseEigenToMsg(initial_tcp_pose,initial_tcp_pose_msg);

  // Assign service arguments
  plan_move_tcp_service.request.group_name = group_name;
  plan_move_tcp_service.request.use_actual_pose = use_actual_pose;
  
  for (int i = 0; i < initial_joint_position.size(); i++ )
  {
    plan_move_tcp_service.request.initial_joint_position.push_back(initial_joint_position[i]); 
  }

  plan_move_tcp_service.request.target_tcp_pose = target_tcp_pose_msg;
  plan_move_tcp_service.request.current_tcp_pose = initial_tcp_pose_msg;
  plan_move_tcp_service.request.velocity_ratio = velocity_ratio;

  if (plan_move_tcp_client_.call(plan_move_tcp_service))
  {
    trajectory = plan_move_tcp_service.response.trajectory;
    ROS_INFO("Success to call service '/flexbotics_manipulation_planner/plan_move_tcp' ");
    ROS_INFO("    plan_move_tcp output variables:");
    ROS_INFO("        success = %d", plan_move_tcp_service.response.success);
    ROS_INFO("        error_id = %d", plan_move_tcp_service.response.error_id);
    ROS_INFO("        message = %s", plan_move_tcp_service.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service '/flexbotics_manipulation_planner/plan_move_tcp' ");
    return false;
  }

  return true;
}

bool getTF(std::string start_frame, std::string end_frame, tf::StampedTransform& transform)
{
  tf::TransformListener listener;

  try {
    // get transformation from camera frame to desired frame
    listener.waitForTransform(end_frame,start_frame,ros::Time(0),ros::Duration(1.0) );
    listener.lookupTransform(end_frame,start_frame,ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // Check
  Eigen::Affine3d tf;
  transformTFToEigen(transform,tf);
  std::vector<double> xyz, rpy;
  eigen_helper_functions::extractXyzRpy(tf, xyz, rpy);
  tf::Quaternion q = tf::createQuaternionFromRPY(rpy[0],rpy[1],rpy[2]);
  ROS_WARN("Transform from %s to %s :  XYZ %3.3f %3.3f %3.3f - RPY %3.3f %3.3f %3.3f - QUAT %f %f %f %f ",
          start_frame.c_str(), end_frame.c_str(), xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2], q[0], q[1], q[2], q[3]);

}

bool ToolStationDetection(
    tool_station_detection_msgs::ToolStationDetection::Request &req,
    tool_station_detection_msgs::ToolStationDetection::Response &res) 
{

  ROS_WARN("NEW TOOL STATION DETECTION REQUEST");

  /// Get variables
  int target_marker_id = req.marker_id;
  std::string group_name = req.group_name;
  geometry_msgs::Pose detection_pose_msg = req.detection_pose;
  double distance_to_marker = req.distance_to_marker;

  Eigen::Affine3d detection_pose;
  tf::poseMsgToEigen(detection_pose_msg, detection_pose);
  double velocity_ratio = req.velocity_ratio;
  
  
  double timeout = 20; // TODO verification ok to leave hard-coded
  std::string marker_number = std::to_string(target_marker_id);
  std::string ar_marker_frame = "ar_marker_" + marker_number;

  ROS_INFO("marker_frame: %s", ar_marker_frame.c_str());
  ROS_INFO("output_frame: %s", output_frame_.c_str());
  

  std::vector<double> initial_joint_position; // not used
  // Create identity matrix (tool center point is end effector of robot)
  Eigen::Affine3d tcp_pose_identity = Eigen::Affine3d::Identity();
  trajectory_msgs::JointTrajectory trajectory_to_detection_pose;
  
  //Plan movement from initial position to detection pose
  if (!PlanMoveTcp(group_name, true, initial_joint_position, detection_pose, tcp_pose_identity, velocity_ratio, trajectory_to_detection_pose))
  {
    ROS_ERROR("Error while planning trajectory in ToolStationDetection!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error while planning trajectory in ToolStationDetection!";
    return true;
  }

  if (!MoveTraj(group_name, trajectory_to_detection_pose , timeout ))
  {
    ROS_ERROR("Error while executing trajectory in ToolStationDetection!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error while executing trajectory in ToolStationDetection!";
    return true;
  }

  // Wait for stable
  sleep(1);

  // Check if desired marker detected
  bool marker_found = false;
  for (int i = 0; i < n_marker_; i++)
  {
    if (target_marker_id == marker_id_[i]) 
      marker_found = true;
    ROS_INFO("Detected marker with id %d", marker_id_[i]);
  }

  if (!marker_found) 
  {
    ROS_ERROR("Error: desired marker not detected!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error: desired marker not detected!";
    return true;
  }

  // Get transformations
  tf::StampedTransform camera_to_output_frame;
  // getTF(camera_frame_,output_frame_, camera_to_output_frame);
  // tf::StampedTransform marker_to_camera_frame;
  // getTF(ar_marker_frame,camera_frame_, marker_to_camera_frame);
  tf::StampedTransform marker_to_output_frame;
  getTF(ar_marker_frame,output_frame_, marker_to_output_frame);


  //refine end effector pose in marker frame (futur parameter ? ) (check desired tool0 position compared to marker)
  std::vector<double> xyz = {0,0,distance_to_marker};
  std::vector<double> rpy = {M_PI,0,0};
  Eigen::Affine3d refine_pose_marker_frame;
  eigen_helper_functions::createHomogeneousMatrixXyzRpy(xyz,rpy,refine_pose_marker_frame);
  geometry_msgs::Pose refine_pose_marker_frame_msg;
  tf::poseEigenToMsg (refine_pose_marker_frame,refine_pose_marker_frame_msg);

   
  geometry_msgs::Pose refine_pose_output_frame_msg;
  geometry_msgs::TransformStamped marker_to_output_frame_geom;
  
  //Transform refine pose output frame
  tf::transformStampedTFToMsg(marker_to_output_frame,marker_to_output_frame_geom);
  tf2::doTransform(refine_pose_marker_frame_msg, refine_pose_output_frame_msg,marker_to_output_frame_geom );

  // To Eigen
  Eigen::Affine3d refine_pose_output_frame;
  tf::poseMsgToEigen(refine_pose_output_frame_msg,refine_pose_output_frame);
  printPose(refine_pose_output_frame,"refine_pose_output_frame");

  

  //Plan movement from detection pose to refine pose 
  trajectory_msgs::JointTrajectory trajectory_to_refine_pose;

  if (!PlanMoveTcp(group_name, true, initial_joint_position, refine_pose_output_frame, tcp_pose_identity, velocity_ratio, trajectory_to_refine_pose))
  {
    ROS_ERROR("Error while planning trajectory in ToolStationDetection!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error while planning trajectory in ToolStationDetection!";
    return true;
  }

  if (!MoveTraj(group_name, trajectory_to_refine_pose , timeout ))
  {
    ROS_ERROR("Error while executing trajectory in ToolStationDetection!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error while executing trajectory in ToolStationDetection!";
    return true;
  }

  // Wait for stable
  sleep(1);

  geometry_msgs::Pose marker_pose_msg;

  // Check if desired marker detected
    for (int i = 0; i < n_marker_; i++)
  {
    if (target_marker_id == marker_id_[i])
    {
      marker_found = true;
      tf::poseEigenToMsg(marker_pose_[i],marker_pose_msg);
    }
      
    ROS_INFO("Detected marker with id %d", marker_id_[i]);
  }

  if (!marker_found) 
  {
    ROS_ERROR("Error: desired marker not detected!");
    res.success = false;
    res.error_id = -1;
    res.message = "Error: desired marker not detected!";
    return true;
  }
  
  res.marker_pose = marker_pose_msg;
  res.success = true;
  res.error_id = 0;
  res.message = "Successful tool detection";

  return true;
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "tool_station_detection_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start();

  nh_ = &nh;
  

  ROS_WARN("Initializing Flexbotics Tool Exchanger Manager Node...");

  if (!initVariables()) 
  {
    ROS_ERROR(
        "Could not initialize Flexbotics Tool Exchanger Manager Node "
        "correctly!!!");
    ROS_WARN("Program finished!!!");
    return -1;
  }

  // Init services
  ROS_INFO("Initializing services...");
  tool_station_detection_service_ =  nh.advertiseService(
      ros::this_node::getName() + "/tool_station_detection", ToolStationDetection);


  // Spin
  ROS_WARN("Waiting for requests!!!");
  ros::waitForShutdown();

  return 0;
}