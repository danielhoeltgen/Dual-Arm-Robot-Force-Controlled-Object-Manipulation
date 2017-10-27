//
// Created by daniel on 27.12.16.
//

#ifndef PROJECT_SCENEMANAGER_H
#define PROJECT_SCENEMANAGER_H

// Dual Arm Toolbox
#include "dual_arm_toolbox/TrajectoryProcessor.h"
#include "dual_arm_toolbox/Transform.h"

// Controller Manager
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <controller_manager_msgs/SwitchController.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Geometry
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

// tf
#include <tf/tf.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// KDL
#include <kdl/frames_io.hpp>
#include <eigen_conversions/eigen_kdl.h>

// Controller Interface
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>

namespace dual_arm_demonstrator_iml {
    class SceneManager {
    protected:
        ros::NodeHandle nh_;

        // Publisher
        ros::Publisher planning_scene_diff_publisher_;

    public:
        SceneManager(ros::NodeHandle& nh);

        // constants
        shape_msgs::SolidPrimitive box_;

        void addBox(std::string id, geometry_msgs::Pose pose, std::string link_name="world", std::string frame_id="world");
        void addShelf(std::string id, geometry_msgs::Pose pose, std::string link_name="world", std::string frame_id="world");
        void setupScene();
        void setupSceneLift();
        void setupSceneLiftCO();
    };
}//namespace

#endif //PROJECT_SCENEMANAGER_H
