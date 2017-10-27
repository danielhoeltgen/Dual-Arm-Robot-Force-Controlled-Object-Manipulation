//
// Created by Daniel Höltgen on 23.12.16.
//

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

// Rviz
#include <moveit_msgs/DisplayTrajectory.h>

// Dual Arm Tools
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

// Dual Arm Demonstrator
#include "dual_arm_demonstrator_iml/DualArmRobot.h"
#include "dual_arm_demonstrator_iml/SceneManager.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "shelf_setup_helper");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    // Scene Setup
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    sceneManager.setupScene();

    // variables
    moveit::planning_interface::MoveGroup::Plan ur5_plan;
    moveit::planning_interface::MoveGroup::Plan ur10_plan;
    moveit::planning_interface::MoveItErrorCode error;
    error.val = -1;

    // move ur5 to shelf
    while (error.val != 1){
        geometry_msgs::PoseStamped ur5_pose;
        ur5_pose.header.frame_id = "shelf";
        ur5_pose.pose.position.x = -0.07;
        ur5_pose.pose.position.y = 0.125;
        ur5_pose.pose.position.z = 0.53;
        KDL::Rotation ur5_rot;  // generated to easily assign quaternion of pose
        ur5_rot.DoRotY(3.14 / 2);
        ur5_rot.GetQuaternion(ur5_pose.pose.orientation.x, ur5_pose.pose.orientation.y, ur5_pose.pose.orientation.z,
                              ur5_pose.pose.orientation.w);
        dualArmRobot.ur5_.setPoseTarget(ur5_pose, dualArmRobot.ur5_.getEndEffectorLink());

        error = dualArmRobot.ur5_.plan(ur5_plan);

    }
    dualArmRobot.execute(ur5_plan);

    error.val = -1;
    while (error.val != 1){
        // move ur10 to shelf
        geometry_msgs::PoseStamped ur10_pose;
        ur10_pose.header.frame_id = "shelf";
        ur10_pose.pose.position.x = 0.125;
        ur10_pose.pose.position.y = 0.25 + 0.07;
        ur10_pose.pose.position.z = 0.53;
        KDL::Rotation ur10_rot;  // generated to easily assign quaternion of pose
        ur10_rot.DoRotX(3.14 / 2);
        ur10_rot.GetQuaternion(ur10_pose.pose.orientation.x, ur10_pose.pose.orientation.y, ur10_pose.pose.orientation.z,
                               ur10_pose.pose.orientation.w);
        dualArmRobot.ur10_.setPoseTarget(ur10_pose, dualArmRobot.ur10_.getEndEffectorLink());

        error = dualArmRobot.ur10_.plan(ur10_plan);

    }
    dualArmRobot.execute(ur10_plan);

    // move closer
    dualArmRobot.graspMove(0.07);

    // END
    ROS_INFO("Place the shelf between the robot arms as displayed in rviz!");
    ros::shutdown();
    return 0;
}