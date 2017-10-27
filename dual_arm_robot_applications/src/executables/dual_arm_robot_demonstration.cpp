//
// Created by Daniel Höltgen on 07.10.16.
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
    sceneManager.setupScene();

    // move home
    dualArmRobot.moveHome();

    // setup constraints
    moveit_msgs::JointConstraint jcm;
    moveit_msgs::Constraints ur5_constraints;
    moveit_msgs::Constraints ur10_constraints;
    moveit_msgs::Constraints both_constraints;

    // when placing box on top ur5 can get blocked because wrist 1 reaches limit
    jcm.joint_name="ur5_wrist_1_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);

    // ur5 can get blocked while placing without this constraint
    jcm.joint_name="ur5_elbow_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.14;
    jcm.tolerance_below = 0.0;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);

    // ur5 sometimes blocks itself when picking the box on bottom, on top ur10 can get problems adapting its trajectory, this solve the issue.
    jcm.joint_name="ur5_shoulder_pan_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 1.5;
    jcm.tolerance_below = 1.5;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);

    // ur10 sometimes blocks itself when picking the box on top, this should solve the issue
    jcm.joint_name="ur10_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 2.5;
    jcm.tolerance_below = 2.5;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);

    jcm.joint_name="ur10_shoulder_pan_joint";
    jcm.position = 0.01;
    jcm.tolerance_above = 0.0;
    jcm.tolerance_below = -3.13;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    both_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);

    dualArmRobot.arms_.setPathConstraints(both_constraints);

    //Eval
    ros::Time before_pick_7;
    ros::Duration manipulation_7;
    ros::Time after_place_7;

    before_pick_7 = ros::Time::now();
    // Pick box7 on top
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "/table_ground";
    direction.vector.x = 0;
    direction.vector.y = 0;
    direction.vector.z = 0.1;
    if (!dualArmRobot.pickBox("box7", direction)) {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }

    // box7 goal pose
    geometry_msgs::Pose box7_goal_pose;
    box7_goal_pose.position.x = 0.02 + sceneManager.box_.dimensions[0]/2;
    box7_goal_pose.position.y = 0.02 + sceneManager.box_.dimensions[1]/2 - 0.005;
    box7_goal_pose.position.z = 0.0155+0.52+0.0155+sceneManager.box_.dimensions[2]/2 + 0.001;
    geometry_msgs::PoseStamped box7_goal_pose_stamped;
    box7_goal_pose_stamped.header.frame_id = "shelf";
    box7_goal_pose_stamped.pose = box7_goal_pose;


    // clear constraints of ur5
    dualArmRobot.ur5_.clearPathConstraints();

    // Place box7
    geometry_msgs::Vector3 go_down;
    go_down.x =0;
    go_down.y =0;
    go_down.z =-0.15;
    if (!dualArmRobot.placeBox("box7", box7_goal_pose_stamped, go_down)){
        ROS_WARN("Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return  0;
    }

    // evaluation
    after_place_7 = ros::Time::now();
    manipulation_7 = after_place_7 - before_pick_7;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 7 took: %li nsec", manipulation_7.toNSec());
    sleep(5);

    dualArmRobot.moveHome();

    // setup constraints
    // ur5 sometimes blocks itself when moving the box on bottom, this should solve the issue
    ur5_constraints.joint_constraints.clear();
    ur10_constraints.joint_constraints.clear();
    both_constraints.joint_constraints.clear();

    jcm.joint_name="ur5_wrist_2_joint";
    jcm.position = 1.5;
    jcm.tolerance_above = 0.6;
    jcm.tolerance_below = 0.6;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // ur10 sometimes blocks itself for path adaption when picking the box on bottom, this should solve the issue
    jcm.joint_name="ur10_wrist_1_joint";
    jcm.position = 0.7;
    jcm.tolerance_above = 1.0;
    jcm.tolerance_below = 1.0;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // when placing box on top ur5 can get blocked because wrist 1 reaches limit
    jcm.joint_name="ur5_wrist_1_joint";
    jcm.position = 0.0;
    jcm.tolerance_above = 3.0;
    jcm.tolerance_below = 3.0;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // ur5 sometimes blocks itself when picking the box on bottom, on top ur10 can get problems adapting its trajectory, this solve the issue.
    jcm.joint_name="ur5_shoulder_pan_joint";
    jcm.position = -2.4;
    jcm.tolerance_above = 2.4;
    jcm.tolerance_below = 0.7;
    jcm.weight = 1.0;
    ur5_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur5_.setPathConstraints(ur5_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    // ur10 sometimes blocks itself when picking the box on top, this should solve the issue
    jcm.joint_name="ur10_wrist_2_joint";
    jcm.position = 0;
    jcm.tolerance_above = 2.5;
    jcm.tolerance_below = 2.5;
    jcm.weight = 1.0;
    ur10_constraints.joint_constraints.push_back(jcm);
    dualArmRobot.ur10_.setPathConstraints(ur10_constraints);
    both_constraints.joint_constraints.push_back(jcm);

    dualArmRobot.arms_.setPathConstraints(both_constraints);


    //Eval
    ros::Time before_pick_3;
    ros::Duration manipulation_3;
    ros::Time after_place_3;

    before_pick_3 = ros::Time::now();

    // Pick box3 on bottom
    geometry_msgs::Vector3Stamped direction2;
    direction2.header.frame_id = "/table_ground";
    direction2.vector.x = 0.20;
    direction2.vector.y = 0.20;
    direction2.vector.z = 0.02;
    if (!dualArmRobot.pickBox("box3", direction2)) {
        ROS_WARN("Pick failed");
        ROS_ERROR("Can't execute demonstration without successful pick. Demonstration aborted.");
        return 0;
    }

    // Place box3 by pushing it to its goal position
    // first, determine box3 goal pose
    geometry_msgs::Pose box3_goal_pose;
    box3_goal_pose.position.x = 0.04 + sceneManager.box_.dimensions[1]/2 /*+ sceneManager.box_.dimensions[1]*/+0.001;
    box3_goal_pose.position.y = 0.04 + sceneManager.box_.dimensions[2]/2 + sceneManager.box_.dimensions[2]+0.001;
    box3_goal_pose.position.z = 0.0155+0.52/2+0.0155/2+sceneManager.box_.dimensions[0]/2+0.005;
    KDL::Rotation box3_goal_rot;
    box3_goal_rot.DoRotY(-3.14/2);
    box3_goal_rot.GetQuaternion(box3_goal_pose.orientation.x, box3_goal_pose.orientation.y, box3_goal_pose.orientation.z, box3_goal_pose.orientation.w);
    geometry_msgs::PoseStamped box3_goal_pose_stamped;
    box3_goal_pose_stamped.header.frame_id = "shelf";
    box3_goal_pose_stamped.pose=box3_goal_pose;

    // second, determine direction for pushing box into goal position
    geometry_msgs::Vector3 direction_push;
    direction_push.x = 0.0;
    direction_push.y = - (sceneManager.box_.dimensions[2]+0.005);// + 0.04);
    direction_push.z = 0.0;

    // setup constraints
    dualArmRobot.ur5_.clearPathConstraints();

    // pre-goal position
    ROS_INFO("moving closer to target position");
    geometry_msgs::PoseStamped ur5_pre_pose = dualArmRobot.ur5_last_goal_pose_;
    ur5_pre_pose.pose.position.z += 0.2;
    if (!dualArmRobot.moveObject("box3", ur5_pre_pose, 0.5)){
        ROS_ERROR("Failed to move box3. Demonstration aborted.");
        return false;
    }

    // start Push Place sequence
    if (!dualArmRobot.pushPlaceBox("box3", box3_goal_pose_stamped, direction_push)){
        ROS_WARN("Push Place Box failed");
        ROS_ERROR("Demonstration aborted to avoid further problems");
        return  0;
    }

    // evaluation
    after_place_3 = ros::Time::now();
    manipulation_3 = after_place_3 - before_pick_3;
    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("manipulation box 3 took: %li nsec", manipulation_3.toNSec());
    sleep(5);
    
    // move robot back into home pose
    dualArmRobot.ur10_.clearPathConstraints();
    dualArmRobot.ur5_.clearPathConstraints();
    dualArmRobot.moveHome();

    // END
    ROS_INFO("Finished demonstration");
    ros::shutdown();
    return 0;
}