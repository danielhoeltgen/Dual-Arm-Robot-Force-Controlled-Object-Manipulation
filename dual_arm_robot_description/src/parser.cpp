#include <urdf/model.h>
#include "ros/ros.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  return 0;
}

