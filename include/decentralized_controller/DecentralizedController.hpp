#pragma once

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>

namespace decentralized_controller {

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
  DecentralizedController(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~DecentralizedController();

  /*!
   * Execute the publishing main cycle of the node.
   */
  void PublishMainLoop(int LoopRate);

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
  //void TopicCallback(const geometry_msgs::PoseWithCovarianceStamped& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  // bool serviceCallback(std_srvs::<Service_Name>::Request& request,
  //                      std_srvs::<Service_Name>::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;
};

} /* namespace */
