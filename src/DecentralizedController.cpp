#include "decentralized_controller/DecentralizedController.hpp"

// STD
#include <string>

namespace decentralized_controller {

  DecentralizedController::DecentralizedController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  ROS_INFO("Successfully launched node.");
}

DecentralizedController::~DecentralizedController()
{
}

void DecentralizedController::PublishMainLoop(int LoopRate)
{

  return;
}

bool DecentralizedController::readParameters()
{

  return true;
}

} /* namespace */
