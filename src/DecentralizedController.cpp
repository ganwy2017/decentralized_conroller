#include "decentralized_controller/DecentralizedController.hpp"

// STD
#include <string>

namespace decentralized_controller {

  DecentralizedController::DecentralizedController(ros::NodeHandle& nodeHandle,const char& robot_id)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  AgentIndx = atoi(&(robot_id));

  std::string robot_id_s = &(robot_id);
  std::string robot_prefix = "/robot";
  std::string PubTopicName = robot_prefix + robot_id_s + "/displacement";
  DispPublisher_ = nodeHandle_.advertise<decentralized_controller::Displacement>(PubTopicName, 100);

  std::string sIndx;
  std::string DispTopicName;
  for( unsigned int nIndx = 1; nIndx <= nNumOfAgents; nIndx++ )
  {
    sIndx = std::to_string(nIndx);
    if( strcmp(sIndx.c_str(), robot_id_s.c_str()) )
    {
      DispTopicName = robot_prefix + sIndx + "/displacement";
      Subscriber_ith_ = nodeHandle_.subscribe(DispTopicName, 5,
                                              &DecentralizedController::TopicCallback, this);
      vDispSubscriber_.push_back(Subscriber_ith_);
    }
  }

  std::string world_frame = "/world";
  displacement.global_frame_id = world_frame;
  displacement.x_disp = vfInitPos[0] - vfFormPos[0];
  displacement.y_disp = vfInitPos[1] - vfFormPos[1];
  realDisp.global_frame_id = world_frame;
  realDisp.x_disp = vfInitPos[0] - vfFormPos[0];
  realDisp.y_disp = vfInitPos[1] - vfFormPos[1];

  // Create the string "robot_name/move_base"
  std::string MoveBaseClientName = robot_prefix + robot_id_s + "/move_base";
  // create the action client
  pActionClient = new MoveBaseClient(MoveBaseClientName, true);
  // Wait for the action server to become available
  ROS_INFO("Waiting for the move_base action server");
  pActionClient->waitForServer(ros::Duration(5));
  ROS_INFO("Connected to move base server");

  ROS_INFO("Successfully launched node.");
}

DecentralizedController::~DecentralizedController()
{
  //delete[] pActionClient;
}

void DecentralizedController::Loop(int LoopRate)
{ 
  std::string robot_ns = "/robot";
  std::string robot_id_s = std::to_string(AgentIndx);
  robot_ns += robot_id_s;
  robot_ns += "_tf";
  std::string robot_frame = tf::resolve(robot_ns, "base_footprint");
  std::string world_frame = "/world";

  ros::Rate   loop_rate(LoopRate);
  int         count   = 0;
  while(ros::ok()) 
  {
    tf::StampedTransform transform;
    try
    {
      //Execute a transfor from the world frame to the robot base footprint frame at the current time
      //and it store it in the StampedTransform object named transform
      ROS_DEBUG("Trying to tranform from frame %s to frame %s", world_frame.c_str(), robot_frame.c_str());
      listener.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform( robot_frame, world_frame,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR(  "Received an exception trying to transform a point from %s to %s: %s",world_frame.c_str(),robot_frame.c_str(), ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    realDisp.x_disp = transform.getOrigin().x() - vfFormPos[0];
    realDisp.y_disp = transform.getOrigin().y() - vfFormPos[1];
    //DispPublisher_.publish(realDisp);
    DispPublisher_.publish(displacement);

    // SENDING THE WAY PINT TO THE MOVE BASE SERVER
    WayPoint.target_pose.header.frame_id = displacement.global_frame_id;
    WayPoint.target_pose.header.stamp = ros::Time::now();

    WayPoint.target_pose.pose.position.x = displacement.x_disp + vfFormPos[0];
    WayPoint.target_pose.pose.position.y = displacement.y_disp + vfFormPos[1];

    // Convert the Euler angle to quaternion
    double radians = 0.0;
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    WayPoint.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to robot no. %s: x = %f, y = %f, theta = %f", robot_id_s.c_str(),
             WayPoint.target_pose.pose.position.x,
             WayPoint.target_pose.pose.position.y, 0.0);
    pActionClient->sendGoal(WayPoint);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return;
}

bool DecentralizedController::readParameters()
{
  if (!nodeHandle_.getParam("number_of_agents", nNumOfAgents)) return false;
  if (!nodeHandle_.getParam("algorithm_gain", fGain)) return false;
  if (!nodeHandle_.getParam("position_in_formation", vfFormPos)) return false;
  if (!nodeHandle_.getParam("initial_position", vfInitPos)) return false;
  return true;
}

void DecentralizedController::TopicCallback(const decentralized_controller::Displacement& message)
{
  float x_disp, y_disp = 0.0;

  x_disp = displacement.x_disp - fGain * (displacement.x_disp - message.x_disp);
  y_disp = displacement.y_disp - fGain * (displacement.y_disp - message.y_disp);

  displacement.x_disp = x_disp;
  displacement.y_disp = y_disp;

  ROS_DEBUG("Updating the algorithm.");
  return;
}


} /* namespace */
