#include "decentralized_controller/DecentralizedController.hpp"

int main(int argc, char** argv)
{

  if (argc < 2) {
    ROS_ERROR("You must specify robot index.");
    return -1;
  }

  const char *robot_id = argv[1];

  ros::init(argc, argv, "decentralized_controller");
  ros::NodeHandle nodeHandle("~");

  decentralized_controller::DecentralizedController DecentralizedController(nodeHandle, *robot_id);

  DecentralizedController.Loop(2);

  ros::spin();
  return 0;
}
