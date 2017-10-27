//
// Created by daniel on 27.12.16.
//

#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <controller_manager/controller_manager.h>
//#include <moveit_ros_control_interface>

// Rviz
#include <moveit_msgs/DisplayTrajectory.h>

// Trajectory tools
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// KDL
#include <kdl/frames_io.hpp>


namespace dual_arm_toolbox{
    class Transform {
    protected:
    public:
        static void transformPoseToKDL(geometry_msgs::Pose pose, KDL::Frame& kdl_frame);
        static void transformKDLtoPose(KDL::Frame kdl_frame, geometry_msgs::Pose& pose);
    };
}

#endif //PROJECT_TRANSFORM_H
