//
// Created by daniel on 27.12.16.
//

#ifndef PROJECT_DUALARMROBOT_H
#define PROJECT_DUALARMROBOT_H

// Robot status
//#define OFFLINE

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
    class DualArmRobot {
    protected:
        ros::NodeHandle nh_;
        std::string ur5_controller_;
        std::string ur10_controller_;
        moveit_msgs::RobotState ur5_last_goal_state_;
        double ee_dist_;    // distance to object because of endeffector size
        KDL::Frame arms_offset_;    // offset between arms
        bool try_again_question();
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    public:
        DualArmRobot(ros::NodeHandle &nh);

        geometry_msgs::PoseStamped ur5_last_goal_pose_;
        geometry_msgs::PoseStamped ur5_last_goal_pose_temp_;

        moveit::planning_interface::MoveGroup ur5_;
        moveit::planning_interface::MoveGroup ur10_;
        moveit::planning_interface::MoveGroup arms_;

        // workaround for moveGroup method does not return attached objects correctly (issue)
        robot_state::RobotState getCurrentRobotState();
        moveit_msgs::RobotState getCurrentRobotStateMsg();

        // calculates a trajectory for both arms based on the trajectory of one arm
        bool adaptTrajectory(moveit_msgs::RobotTrajectory ur5_trajectory, KDL::Frame offset,
                             moveit_msgs::RobotTrajectory &both_arms_trajectory, double jump_threshold = 0.4);

        // returns the Offset-Vector between both end effectors
        KDL::Frame getCurrentOffset();

        // executes a pick: Moves both arms to the object, grasps it, moves up
        bool pickBox(std::string object_id, geometry_msgs::Vector3Stamped lift_direction);

        // pace Methods
        bool placeBox(std::string object_id, geometry_msgs::PoseStamped ur5_place_pose,
                      geometry_msgs::Vector3 close_direction);

        bool pushPlaceBox(std::string object_id, geometry_msgs::PoseStamped box_pose, geometry_msgs::Vector3 direction);

        bool moveObject(std::string object_id, geometry_msgs::PoseStamped ur5_pose, double scale=0.2);
        bool planMoveObject(std::string object_id, geometry_msgs::PoseStamped ur5_pose, double scale=0.2); // Only plan an visualize without executing. For use in validation.

        // enable/disable collision check between robot arms
        void allowedArmCollision(bool enable, std::string ur5_attachedObject);

        bool place(geometry_msgs::Pose pose_object);

        bool switch_controller(std::string stop_name, std::string start_name, std::string ur_namespace);

        bool graspMove(double distance, bool avoid_collisions = true, bool use_ur5=true, bool use_ur10=true); //both arms will be moved closer together

        bool linearMove(geometry_msgs::Vector3Stamped direction, bool avoid_collisions = true, bool use_ur5=true, bool use_ur10=true);

        bool linearMoveParallel(geometry_msgs::Vector3Stamped direction, std::string object_id, double traj_scale=1, bool avoid_collisions = true);

        bool execute(moveit::planning_interface::MoveGroup::Plan plan);

        bool moveHome();
    };
}//ns
#endif //PROJECT_DUALARMROBOT_H
