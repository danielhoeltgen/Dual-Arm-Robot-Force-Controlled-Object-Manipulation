//
// Created by daniel on 24.03.17.
//

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

//#include "../../include/ur_logging/UrLogger.h"
#include "ur_logging/UrLogger.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "dual_arm_robot_logger");
    ros::NodeHandle nh;

    ros::AsyncSpinner asyncSpinner(1);
    asyncSpinner.start();

    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("ur5");
    ur_namespaces.push_back("ur10");
    UR_Logger ur_logger(nh, ur_namespaces);

    ur_logger.start(20);
    ros::Duration(6).sleep();
    ur_logger.stop();

    ros::shutdown();
    return 0;
}