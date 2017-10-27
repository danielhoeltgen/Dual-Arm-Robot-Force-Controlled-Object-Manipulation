//
// Created by daniel on 27.12.16.
//

#ifndef PROJECT_TRAJECTORYPROCESSOR_H
#define PROJECT_TRAJECTORYPROCESSOR_H

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


namespace dual_arm_toolbox {
    class TrajectoryProcessor {
    protected:
    public:
        // fuses two trajectories into one; both should be of same length
        static bool fuse(moveit_msgs::RobotTrajectory &arms_trajectory,
                         moveit_msgs::RobotTrajectory arm1_trajectory,
                         moveit_msgs::RobotTrajectory arm2_trajectory);

        // splits one trajectory for both arms into two trajectory for each arm
        static bool split(moveit_msgs::RobotTrajectory arms_trajectory,
                          moveit_msgs::RobotTrajectory &arm1_trajectory,
                          moveit_msgs::RobotTrajectory &arm2_trajectory,
                          std::string arm1_prefix,
                          std::string arm2_prefix);

        // moveit sometimes generates trajectories with two time the same time stamp. This method eliminates one entry.
        static void clean(moveit_msgs::RobotTrajectory &trajectory);

        static void computeTimeFromStart(moveit_msgs::RobotTrajectory &trajectory, double step_t);

        static void scaleTrajectorySpeed(moveit_msgs::RobotTrajectory &trajectory, double scale);

        static bool
        computeVelocities(moveit_msgs::RobotTrajectory &trajectory, moveit::planning_interface::MoveGroup &moveGroup);

        static void visualizePlan(moveit::planning_interface::MoveGroup::Plan &plan, unsigned int sec);
    };
}//namespace
#endif //PROJECT_TRAJECTORYPROCESSOR_H
