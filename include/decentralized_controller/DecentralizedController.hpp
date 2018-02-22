#pragma once

#include "decentralized_controller/Displacement_msgs.h"

// ROS
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

namespace decentralized_controller {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class DecentralizedController
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  DecentralizedController(ros::NodeHandle& nodeHandle, const char &robot_id); //: ActionClient("ActionName",true)

  /*!
   * Destructor.
   */
  virtual ~DecentralizedController();

  /*!
   * Execute the publishing main cycle of the node.
   */
  void Loop(double LoopRate);

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void TopicCallback(const decentralized_controller::Displacement_msgs& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

	//! ROS topic publisher for the displacement of my robot position and the formation desired one.
	ros::Publisher DispPublisher_;

  //! ROS topic subscriber for the displacement of the i-th robot.
  ros::Subscriber Subscriber_ith_;

  //! vector of ROS topic subscriber for the displacement of the other robot.
  std::vector<ros::Subscriber> vDispSubscriber_;

  //! Local version of the displacement message to deliver, obtained with the measured distances
  decentralized_controller::Displacement_msgs realDisp;

  //! Local version of the displacement message to deliver, obtained as the discrete evolution of the algorithm
  decentralized_controller::Displacement_msgs displacement;

  //! Move base client to move the robot
  MoveBaseClient* pActionClient;

  //! Way point to reach
  move_base_msgs::MoveBaseGoal WayPoint;

  //! Buffer for the Transformation between the global frame /world and the robot postion
  tf::TransformListener listener;

  //! Position in the formation of the agent
  std::vector<float> vfFormPos;

  //! Initial Position of the Agent
  std::vector<float> vfInitPos;

  //! Gain for the algorithm
  float fGain;

  //! Number of Agents
  int nNumOfAgents;

  //! The index of this agent
  int AgentIndx;
};

} /* namespace */
