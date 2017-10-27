//
// Created by Daniel Höltgen on 08.04.17.
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
    ros::init(argc, argv, "dual_arm_robot_demonstration");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle nh;

    #ifdef OFFLINE
    ROS_WARN("Robot offline");
    #endif

    // Dual Arm Robot Setup
    dual_arm_demonstrator_iml::DualArmRobot dualArmRobot(nh);

    // Scene Setup
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh);
    // with or without collision objects?
    sceneManager.setupSceneLiftCO();
//    sceneManager.setupSceneLift();

    // move home
    dualArmRobot.moveHome();

    // setup constraints
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints ur5_constraints;
    moveit_msgs::Constraints ur10_constraints;
    jcm.joint_name="ur10_shoulder_pan_joint";
    jcm.position = 2.4;
    jcm.tolerance_above = 0.7;
    jcm.tolerance_below = 2.5;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);

    /*
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints ur5_constraints;
    moveit_msgs::Constraints ur10_constraints;
    jcm.joint_name="ur10_shoulder_pan_joint";
    jcm.position = -2.4;
    jcm.tolerance_above = 2.5;
    jcm.tolerance_below = 0.7;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);*/
    /*
    jcm.joint_name="ur10_wrist_2_joint";
    jcm.position = 3.0;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 4.0;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);*/
/*
    jcm.joint_name="ur5_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 1.0;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);*/

    jcm.joint_name="ur5_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 0.4;
    jcm.tolerance_below = 0.4;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);

    // Pick box1
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "/table_ground";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.1;
    if (!dualArmRobot.pickBox("box1", direction)) {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }

    // clear path constraints
    dualArmRobot.ur5_.clearPathConstraints();
    dualArmRobot.ur10_.clearPathConstraints();

    // plan move up
    geometry_msgs::PoseStamped start_pose = dualArmRobot.ur5_.getCurrentPose(dualArmRobot.ur5_.getEndEffectorLink());
    geometry_msgs::PoseStamped lift_pose = start_pose;
    lift_pose.pose.position.z = lift_pose.pose.position.z +0.6;

    for (unsigned int i =0; i < 15 ; i++){
        ROS_INFO(":::::: START EVALUATION %i::::::", i);
        dualArmRobot.planMoveObject("box1", lift_pose, 0.2);
        ROS_INFO(":::::: END EVALUATION %i::::::", i);
    }


    // END
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}