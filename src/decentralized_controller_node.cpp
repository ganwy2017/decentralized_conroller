#include <ros/ros.h>
#include "decentralized_controller/DecentralizedController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decentralized_controller");
  ros::NodeHandle nodeHandle("~");

  decentralized_controller::DecentralizedController DecentralizedController(nodeHandle);

  DecentralizedController.PublishMainLoop(10);

  ros::spin();
  return 0;
}
