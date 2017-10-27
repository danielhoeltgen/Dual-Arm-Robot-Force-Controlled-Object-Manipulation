//
// Created by Daniel Höltgen on 20.10.16.
//

#include <dual_arm_demonstrator_iml/SceneManager.h>
#include "dual_arm_demonstrator_iml/DualArmRobot.h"

using namespace dual_arm_demonstrator_iml;

DualArmRobot::DualArmRobot(ros::NodeHandle& nh) :
        ur5_("ur5_manipulator"),
        ur10_("ur10_manipulator"),
        arms_("arms"),
        nh_(nh) {
    // MoveIt! Setup
    //ur5_.setPlanningTime(40);
    ur5_.setPlanningTime(5);
    ur5_.setNumPlanningAttempts(25);
    //ur10_.setPlanningTime(40);
    ur10_.setPlanningTime(5);
    ur10_.setNumPlanningAttempts(25);

    // setup planner
    ur5_.setPlannerId("RRTConnectkConfigDefault");
    ur10_.setPlannerId("RRTConnectkConfigDefault");

    // planning scene monitor
    planning_scene_monitor_ = boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    // initialize ur5 specific variables
    ur5_last_goal_pose_ = ur5_.getCurrentPose(ur5_.getEndEffectorLink());
    ur5_last_goal_state_ = getCurrentRobotStateMsg();

    // Controller Interface
    /*
#ifdef SIMULATION
    ur5_controller_ = "fake_ur5_manipulator_controller";
    ur10_controller_ = "fake_ur10_manipulator_controller";
#endif
#ifndef SIMULATION*/
    ur5_controller_ = "ur5/ur5_vel_based_pos_traj_controller";
    ur10_controller_ = "ur10/ur10_vel_based_pos_traj_controller";
//#endif
}

robot_state::RobotState DualArmRobot::getCurrentRobotState(){
    planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    robot_state::RobotState current_state = ps->getCurrentState();
    return current_state;
}

moveit_msgs::RobotState DualArmRobot::getCurrentRobotStateMsg() {
    moveit_msgs::RobotState current_state_msg;
    moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), current_state_msg);
    return current_state_msg;
}

KDL::Frame DualArmRobot::getCurrentOffset(){
    geometry_msgs::PoseStamped ur5_pose = ur5_.getCurrentPose(ur5_.getEndEffectorLink());
    geometry_msgs::PoseStamped ur10_pose = ur10_.getCurrentPose(ur10_.getEndEffectorLink());
    KDL::Frame ur5_frame;
    KDL::Frame ur10_frame;
    dual_arm_toolbox::Transform::transformPoseToKDL(ur5_pose.pose, ur5_frame);
    dual_arm_toolbox::Transform::transformPoseToKDL(ur10_pose.pose, ur10_frame);
    KDL::Frame offset;
    offset = ur5_frame.Inverse() * ur10_frame;
    return offset;
}

bool DualArmRobot::adaptTrajectory(moveit_msgs::RobotTrajectory ur5_trajectory, KDL::Frame offset, moveit_msgs::RobotTrajectory& both_arms_trajectory, double jump_threshold){
    // setup planning scene
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planningScene(kinematic_model);

    // setup JointModelGroup
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* ur5_joint_model_group = kinematic_model->getJointModelGroup("ur5_manipulator");
    const robot_state::JointModelGroup* ur10_joint_model_group = kinematic_model->getJointModelGroup("ur10_manipulator");

    // setup both_arms_trajectory
    both_arms_trajectory = ur5_trajectory;
    for (unsigned int j=0; j < ur10_joint_model_group->getActiveJointModelNames().size(); j++){
        both_arms_trajectory.joint_trajectory.joint_names.push_back(ur10_joint_model_group->getActiveJointModelNames()[j]);
    }

    // ik service client setup
    ros::ServiceClient ik_client = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    moveit_msgs::GetPositionIK ik_msg;

    // current state for initial seed
    //moveit_msgs::RobotState robot_state;
    //moveit::core::robotStateToRobotStateMsg(*arms_.getCurrentState(), robot_state);
    //ik_msg.request.ik_request.robot_state = robot_state;
    ik_msg.request.ik_request.robot_state = getCurrentRobotStateMsg();
    ik_msg.request.ik_request.attempts = 5;
    ik_msg.request.ik_request.avoid_collisions = true;
    ik_msg.request.ik_request.group_name = ur10_.getName();
    ik_msg.request.ik_request.timeout = ros::Duration(15);

    // computing values for trajectory
    for (unsigned int i = 0; i < ur5_trajectory.joint_trajectory.points.size(); i++){
        std::vector<double> ur5_joint_values;
        ur5_joint_values.clear();
        for (unsigned int a = 0; a < ur5_trajectory.joint_trajectory.points[i].positions.size(); a++){
            ur5_joint_values.push_back(ur5_trajectory.joint_trajectory.points[i].positions[a]);
        }
        // fk -> pos ur5
        KDL::Frame frame_pose_ur5;
        kinematic_state->setJointGroupPositions(ur5_joint_model_group, ur5_joint_values);
        const Eigen::Affine3d &end_effector_pose_ur5 = kinematic_state->getGlobalLinkTransform(ur5_.getEndEffectorLink());
        tf::transformEigenToKDL(end_effector_pose_ur5, frame_pose_ur5);

        // compute pos ur10
        KDL::Frame frame_pose_ur10 = frame_pose_ur5*offset;

        // ik msg for request for ur10
        ik_msg.request.ik_request.pose_stamped.header.frame_id = "table_ground";
        ik_msg.request.ik_request.pose_stamped.pose.position.x = frame_pose_ur10.p.x();
        ik_msg.request.ik_request.pose_stamped.pose.position.y = frame_pose_ur10.p.y();
        ik_msg.request.ik_request.pose_stamped.pose.position.z = frame_pose_ur10.p.z();
        frame_pose_ur10.M.GetQuaternion(
                ik_msg.request.ik_request.pose_stamped.pose.orientation.x,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.y,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.z,
                ik_msg.request.ik_request.pose_stamped.pose.orientation.w
        );

        // get ik solution
        bool try_again;
        double try_count = 0;
        do {
            ik_client.call(ik_msg.request, ik_msg.response);
            if (ik_msg.response.error_code.val != 1) {
                ROS_WARN("ik request error");
                return false;
            }
            // try again if jump is too huge
            try_again = false;
            for (unsigned int a = 0; a < ik_msg.response.solution.joint_state.position.size(); a++){
                try_again = try_again || (std::abs(ik_msg.response.solution.joint_state.position[a] - ik_msg.request.ik_request.robot_state.joint_state.position[a]) > jump_threshold);
            }
            if (try_again) {
                ROS_INFO("Ik: jump detected. One value deviates more than %f. Trying again", jump_threshold);
                try_count++;
                if (try_count > 10){
                    ROS_WARN("could not find solution without jump");
                    return false;
                }
            }
        } while (try_again);

        // write results into trajectory msg
        for (unsigned int j=0; j < ik_msg.response.solution.joint_state.position.size(); j++){
            both_arms_trajectory.joint_trajectory.points[i].positions.push_back(ik_msg.response.solution.joint_state.position[j]);
        }

        // use result as seed for next calculation
        ik_msg.request.ik_request.robot_state = ik_msg.response.solution;
    }

    // execution speed
    dual_arm_toolbox::TrajectoryProcessor::computeTimeFromStart(both_arms_trajectory, 0.4);

    // compute velocities for trajectory
    return dual_arm_toolbox::TrajectoryProcessor::computeVelocities(both_arms_trajectory, arms_);
}

bool DualArmRobot::switch_controller(std::string stop_name, std::string start_name, std::string ur_namespace) {
    ROS_INFO("Switching controllers");
#ifndef OFFLINE
    // setup
    ros::ServiceClient srv_switch_controller = nh_.serviceClient<controller_manager_msgs::SwitchController>(ur_namespace+"/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController switchController;
    switchController.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    sleep(2);

    // stop
    switchController.request.stop_controllers.push_back(stop_name);
    bool success_stop = srv_switch_controller.call(switchController);
    ROS_INFO("Stopping controller %s",success_stop?"SUCCEDED":"FAILED");
    if (!success_stop) return false;

    // clear
    switchController.request.stop_controllers.clear();

    // start admittance controller
    switchController.request.BEST_EFFORT;
    switchController.request.start_controllers.push_back(start_name);
    bool success_start = srv_switch_controller.call(switchController);
    ROS_INFO("Starting controller %s",success_start?"SUCCEDED":"FAILED");
    switchController.request.start_controllers.clear();

    // Switch controller in moveit-interface
    if (success_start){
        if (ur5_controller_.compare(0,ur_namespace.size(),ur_namespace)==0){
            ur5_controller_ = ur_namespace+"/"+start_name;
        }
        if (ur10_controller_.compare(0,ur_namespace.size(),ur_namespace)==0){
            ur10_controller_ = ur_namespace+"/"+start_name;
        }
    }

    return success_start;
#endif
#ifdef OFFLINE
    return true;
#endif
}

bool DualArmRobot::graspMove(double distance, bool avoid_collisions, bool use_ur5, bool use_ur10) {
    bool try_step;
    double fraction;

    // move closer with ur10 and ur5 by using cartesian path
    if (distance > 0) ROS_INFO("Moving towards object");
    if (distance < 0) ROS_INFO("Moving away from object");

    // ur5
    if (use_ur5) try_step = true;
    while (try_step && ros::ok()) {
        ur5_.setStartState(ur5_last_goal_state_);
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint = ur5_last_goal_pose_.pose;
        ur5_waypoints.push_back(ur5_waypoint);

        // transform distance vector
        KDL::Vector ur5_vec_d;  // distance vector
        KDL::Frame ur5_p_eef;   // endeffector frame
        dual_arm_toolbox::Transform::transformPoseToKDL(ur5_waypoint, ur5_p_eef);
        ur5_vec_d.x(0);
        ur5_vec_d.y(0);
        ur5_vec_d.z(distance);
        ur5_vec_d = ur5_p_eef.M * ur5_vec_d;     // Rotate distance vector

        // calculate waypoint
        ur5_waypoint.position.x = ur5_waypoint.position.x + ur5_vec_d.x();
        ur5_waypoint.position.y = ur5_waypoint.position.y + ur5_vec_d.y();
        ur5_waypoint.position.z = ur5_waypoint.position.z + ur5_vec_d.z();

        ur5_waypoints.push_back(ur5_waypoint);
        moveit_msgs::RobotTrajectory ur5_trajectory;
        fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_trajectory);
            moveit::planning_interface::MoveGroup::Plan ur5_plan;
            ur5_plan.trajectory_ = ur5_trajectory;
            execute(ur5_plan);
            try_step = false;
        }
    }

    // ur10
    if (use_ur10) try_step = true;
    while (try_step && ros::ok()) {
        ur10_.setStartStateToCurrentState();
        std::vector<geometry_msgs::Pose> ur10_waypoints;
        geometry_msgs::Pose ur10_waypoint = ur10_.getCurrentPose(ur10_.getEndEffectorLink()).pose;
        ur10_waypoints.push_back(ur10_waypoint);

        // transform distance vector
        KDL::Vector ur10_vec_d;  // distance vector
        KDL::Frame ur10_p_eef;   // endeffector frame
        dual_arm_toolbox::Transform::transformPoseToKDL(ur10_waypoint, ur10_p_eef);
        ur10_vec_d.x(0);
        ur10_vec_d.y(0);
        ur10_vec_d.z(distance);
        ur10_vec_d = ur10_p_eef.M * ur10_vec_d;     // Rotate distance vector

        // calculate waypoint
        ur10_waypoint.position.x = ur10_waypoint.position.x + ur10_vec_d.x();
        ur10_waypoint.position.y = ur10_waypoint.position.y + ur10_vec_d.y();
        ur10_waypoint.position.z = ur10_waypoint.position.z + ur10_vec_d.z();

        ur10_waypoints.push_back(ur10_waypoint);
        moveit_msgs::RobotTrajectory ur10_trajectory;
        fraction = ur10_.computeCartesianPath(ur10_waypoints, 0.001, 0.0, ur10_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("UR10 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            moveit::planning_interface::MoveGroup::Plan ur10_plan;
            dual_arm_toolbox::TrajectoryProcessor::clean(ur10_trajectory);
            ur10_plan.trajectory_ = ur10_trajectory;
            execute(ur10_plan);
            try_step = false;
        }
    }

    return true;
}

bool DualArmRobot::try_again_question() {
    // ask to try again
    /*
    std::cout << "Try this step again? (y/n) ";
    char response;
    std::cin >> response;
    if (response == 'n') return false;
    else return true;*/

    // automatic retry
    if (ros::ok()){
        ROS_WARN("Trying this step again. Press Ctr+C to abort.");
        return true;
    }
    return false;
}

bool DualArmRobot::pickBox(std::string object_id , geometry_msgs::Vector3Stamped lift_direction) {
    ROS_INFO("Starting Pick Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // move ur5
    /*
    try_step = true;
    while (try_step && ros::ok()) {
        geometry_msgs::PoseStamped ur5_pose;
        ur5_pose.header.frame_id = object_id;
        ur5_pose.pose.position.x = -0.18;
        ur5_pose.pose.position.y = 0.0;
        ur5_pose.pose.position.z = 0.0;
        KDL::Rotation ur5_rot;  // generated to easily assign quaternion of pose
        ur5_rot.DoRotY(3.14 / 2);
        ur5_rot.GetQuaternion(ur5_pose.pose.orientation.x, ur5_pose.pose.orientation.y, ur5_pose.pose.orientation.z,
                              ur5_pose.pose.orientation.w);
        ur5_.setStartState(ur5_last_goal_state_);
        ur5_.setPoseTarget(ur5_pose);
        moveit::planning_interface::MoveGroup::Plan ur5_plan;
        error = ur5_.plan(ur5_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur5 into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur5_plan.trajectory_, 0.4);

            execute(ur5_plan);
            try_step = false;
        }
    }*/

    // move ur10
    /*
    try_step = true;
    while (try_step && ros::ok()) {
        geometry_msgs::PoseStamped ur10_pose;
        ur10_pose.header.frame_id = object_id;
        ur10_pose.pose.position.x = 0.18;
        ur10_pose.pose.position.y = 0;
        ur10_pose.pose.position.z = 0;
        KDL::Rotation ur10_rot;  // generated to easily assign quaternion of pose
        ur10_rot.DoRotY(3.14 / 2);
        ur10_rot.DoRotX(3.14);
        ur10_rot.GetQuaternion(ur10_pose.pose.orientation.x, ur10_pose.pose.orientation.y, ur10_pose.pose.orientation.z,
                               ur10_pose.pose.orientation.w);
        ur10_.setPoseTarget(ur10_pose);

        moveit::planning_interface::MoveGroup::Plan ur10_plan;
        error = ur10_.plan(ur10_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur10 into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(ur10_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur10_plan.trajectory_, 0.4);
            execute(ur10_plan);
            try_step = false;
        }
    }*/

    // move both arms
    try_step = true;
    while (try_step && ros::ok()) {
        geometry_msgs::PoseStamped ur5_pose;
        ur5_pose.header.frame_id = object_id;
        ur5_pose.pose.position.x = -0.18;
        ur5_pose.pose.position.y = 0.0;
        ur5_pose.pose.position.z = 0.0;
        KDL::Rotation ur5_rot;  // generated to easily assign quaternion of pose
        ur5_rot.DoRotY(3.14 / 2);
        ur5_rot.GetQuaternion(ur5_pose.pose.orientation.x, ur5_pose.pose.orientation.y, ur5_pose.pose.orientation.z,
                              ur5_pose.pose.orientation.w);

        geometry_msgs::PoseStamped ur10_pose;
        ur10_pose.header.frame_id = object_id;
        ur10_pose.pose.position.x = 0.18;
        ur10_pose.pose.position.y = 0;
        ur10_pose.pose.position.z = 0;
        KDL::Rotation ur10_rot;  // generated to easily assign quaternion of pose
        ur10_rot.DoRotY(3.14 / 2);
        ur10_rot.DoRotX(3.14);
        ur10_rot.GetQuaternion(ur10_pose.pose.orientation.x, ur10_pose.pose.orientation.y, ur10_pose.pose.orientation.z,
                               ur10_pose.pose.orientation.w);

        arms_.setStartState(ur5_last_goal_state_);
        arms_.setPoseTarget(ur5_pose, ur5_.getEndEffectorLink());
        arms_.setPoseTarget(ur10_pose, ur10_.getEndEffectorLink());

        moveit::planning_interface::MoveGroup::Plan arms_plan;
        error = arms_.plan(arms_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur5 into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);

            execute(arms_plan);
            try_step = false;
        }
    }


    // Grasp
    // move closer with ur10 and ur5 by using cartesian path
    //if (!graspMove(0.075)) {
    if (!graspMove(0.075)) {
        ROS_WARN("failed moving closer to object");
        return false;
    };

    // attach object to ur and update State Msg
    ur5_.attachObject(object_id, ur5_.getEndEffectorLink());
    ur5_last_goal_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // compute cartesian Path for ur5
    ur5_.setStartState(ur5_last_goal_state_);
    moveit_msgs::RobotTrajectory ur5_trajectory;
    try_step = true;
    while (try_step && ros::ok()) {
        ur5_.setPoseReferenceFrame(lift_direction.header.frame_id);
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint = ur5_.getCurrentPose(ur5_.getEndEffectorLink()).pose;
        ur5_waypoints.push_back(ur5_waypoint);
        ur5_waypoint.position.x = ur5_waypoint.position.x + lift_direction.vector.x;
        ur5_waypoint.position.y = ur5_waypoint.position.y + lift_direction.vector.y;
        ur5_waypoint.position.z = ur5_waypoint.position.z + lift_direction.vector.z;
        //ur5_last_goal_pose_temp_.pose = ur5_waypoint; // TODO: find better solution
        //ur5_last_goal_pose_temp_.header.frame_id = "table_ground";
        ur5_waypoints.push_back(ur5_waypoint);
        double fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, false);
        if (fraction < 1) {
            ROS_INFO("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Pick can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    if (adaptTrajectory(ur5_trajectory, arms_offset_, both_arms_trajectory)) ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        allowedArmCollision(false,object_id);
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    allowedArmCollision(false,object_id);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // Grasp by switching controller and wait for contact while visualizing plan
    if (!switch_controller("ur5_vel_based_pos_traj_controller", "ur5_vel_based_admittance_traj_controller", "ur5"))
        ROS_WARN("failed switching controller");

    // visualize plan
    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);

    ROS_INFO("executing plan");
    execute(both_arms_plan);
    return true;
}

bool DualArmRobot::linearMoveParallel(geometry_msgs::Vector3Stamped direction, std::string object_id, double traj_scale, bool avoid_collisions) {
    // compute cartesian Path for ur5
    moveit_msgs::RobotTrajectory ur5_trajectory;
    bool try_step = true;
    while (try_step && ros::ok()) {
        ur5_.setPoseReferenceFrame(direction.header.frame_id);
        ur5_.setStartState(ur5_last_goal_state_);
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint = ur5_last_goal_pose_.pose;
        ur5_waypoints.push_back(ur5_waypoint);
        ur5_waypoint.position.x = ur5_waypoint.position.x + direction.vector.x;
        ur5_waypoint.position.y = ur5_waypoint.position.y + direction.vector.y;
        ur5_waypoint.position.z = ur5_waypoint.position.z + direction.vector.z;
        ur5_waypoints.push_back(ur5_waypoint);
        double fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, false);
        if (fraction < 1) {
            ROS_INFO("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Linear parallel move can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(ur5_trajectory);
    // get both arms trajectory
    arms_offset_ = getCurrentOffset();
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    if (adaptTrajectory(ur5_trajectory, arms_offset_, both_arms_trajectory)) ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        allowedArmCollision(false,object_id);
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);
    allowedArmCollision(false,object_id);

    // scale trajectory
    dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(both_arms_trajectory, traj_scale);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // visualize plan
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 0.1);

    ROS_INFO("executing plan");
    execute(both_arms_plan);
    return true;
}

bool DualArmRobot::placeBox(std::string object_id, geometry_msgs::PoseStamped box_place_pose,
                            geometry_msgs::Vector3 close_direction) {
    ROS_INFO("Starting Place Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // calculate targets
    geometry_msgs::PoseStamped ur5_target;
    ur5_target = box_place_pose;
    KDL::Rotation ur5_rot;  // generated to easily assign quaternion of pose
    ur5_rot.DoRotY(3.14 / 2);
    ur5_rot.GetQuaternion(ur5_target.pose.orientation.x, ur5_target.pose.orientation.y, ur5_target.pose.orientation.z,
                          ur5_target.pose.orientation.w);
    KDL::Vector shift;
    SceneManager sceneManager(nh_);
    shift.z(-sceneManager.box_.dimensions[0]/2);
    shift = ur5_rot*shift;
    ur5_target.pose.position.x = box_place_pose.pose.position.x - close_direction.x + shift.x();
    ur5_target.pose.position.y = box_place_pose.pose.position.y - close_direction.y + shift.y();
    ur5_target.pose.position.z = box_place_pose.pose.position.z - close_direction.z + shift.z();

    // target position before placing
    moveObject(object_id, ur5_target);

    // compute cartesian Path for ur5
    moveit_msgs::RobotTrajectory ur5_trajectory_2;
    try_step = true;
    while (try_step && ros::ok()) {
        allowedArmCollision(true, object_id);
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        ur5_.setStartState(ur5_last_goal_state_);
        geometry_msgs::Pose ur5_waypoint = ur5_last_goal_pose_.pose;
        ur5_waypoints.push_back(ur5_waypoint);
        ur5_waypoint.position.x = ur5_waypoint.position.x + close_direction.x;
        ur5_waypoint.position.y = ur5_waypoint.position.y + close_direction.y;
        ur5_waypoint.position.z = ur5_waypoint.position.z + close_direction.z;
        ur5_waypoints.push_back(ur5_waypoint);
        double fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory_2, false);
        if (fraction < 1) {
            ROS_INFO("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            ROS_INFO("Fraction is less than 100%%. Insufficient for dual-arm configuration. Pick can not be executed");
            try_step = try_again_question();
            if (!try_step){
                allowedArmCollision(false,object_id);
                return false;
            }
        }
        else try_step = false;
    }

    // get both arms trajectory
    moveit_msgs::RobotTrajectory both_arms_trajectory_2;
    if (adaptTrajectory(ur5_trajectory_2, arms_offset_, both_arms_trajectory_2, 1.0)) ROS_INFO("successfully calculated trajectory for both arms");
    else {
        ROS_WARN("Problem adapting trajectory");
        return false;
    }
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory_2);
    allowedArmCollision(false,object_id);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory_2;
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // execute
    execute(both_arms_plan);

    // release clamp
    // switch controller
    if (!switch_controller("ur5_vel_based_admittance_traj_controller", "ur5_vel_based_pos_traj_controller", "ur5"))
        ROS_WARN("failed switching controller");
    sleep(0.5); // to be sure robot is at goal position

    // correct pose. Important for box to be at the right place in simulation
    geometry_msgs::Vector3Stamped correct_vec;
    correct_vec.header.frame_id = ur5_last_goal_pose_.header.frame_id;
    geometry_msgs::PoseStamped curr_pose = ur5_.getCurrentPose(ur5_.getEndEffectorLink());
    correct_vec.vector.x = ur5_last_goal_pose_.pose.position.x - curr_pose.pose.position.x;
    correct_vec.vector.y = ur5_last_goal_pose_.pose.position.y - curr_pose.pose.position.y;
    correct_vec.vector.z = ur5_last_goal_pose_.pose.position.z - curr_pose.pose.position.z;
    ur5_last_goal_pose_ = curr_pose;
    ur5_last_goal_state_ = getCurrentRobotStateMsg();
    linearMove(correct_vec, false, true, false);

    // detach object
    ur5_.detachObject(object_id);
    ur5_last_goal_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // un-grasp
    graspMove(-0.08, false);

    return true;
}

bool DualArmRobot::pushPlaceBox(std::string object_id, geometry_msgs::PoseStamped box_pose, geometry_msgs::Vector3 direction) {
    ROS_INFO("Starting Push Place Box sequence");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    dual_arm_demonstrator_iml::SceneManager sceneManager(nh_);

    // calculate ur5 pose from box goal pose
    geometry_msgs::PoseStamped ur5_target_pose;
    ur5_target_pose = box_pose;
    KDL::Rotation ur5_rot;
    ur5_rot = ur5_rot.Quaternion(ur5_target_pose.pose.orientation.x, ur5_target_pose.pose.orientation.y, ur5_target_pose.pose.orientation.z, ur5_target_pose.pose.orientation.w);
    ur5_rot.DoRotY(-3.14/2);
    KDL::Vector ur5_shift;
    ur5_shift.z(-sceneManager.box_.dimensions[0]/2);
    ur5_shift = ur5_rot * ur5_shift;
    ur5_target_pose.pose.position.x = ur5_target_pose.pose.position.x - direction.x + ur5_shift.x();
    ur5_target_pose.pose.position.y = ur5_target_pose.pose.position.y - direction.y + ur5_shift.y();
    ur5_target_pose.pose.position.z = ur5_target_pose.pose.position.z - direction.z + ur5_shift.z();
    ur5_rot.GetQuaternion(ur5_target_pose.pose.orientation.x, ur5_target_pose.pose.orientation.y, ur5_target_pose.pose.orientation.z, ur5_target_pose.pose.orientation.w);

    // ur5 target position for placing
    ROS_INFO("moving object with both arms");
    if (!moveObject(object_id, ur5_target_pose, 0.1)) return false;

    // release clamp
    // switch controller
    if (!switch_controller("ur5_vel_based_admittance_traj_controller", "ur5_vel_based_pos_traj_controller", "ur5"))
        ROS_WARN("failed switching controller");
    sleep(0.5); // to be sure robot is at goal position

    // correct pose. Important for box to be at the right place in simulation
    geometry_msgs::Vector3Stamped correct_vec;
    correct_vec.header.frame_id = ur5_last_goal_pose_.header.frame_id;
    geometry_msgs::PoseStamped curr_pose = ur5_.getCurrentPose(ur5_.getEndEffectorLink());
    correct_vec.vector.x = ur5_last_goal_pose_.pose.position.x - curr_pose.pose.position.x;
    correct_vec.vector.y = ur5_last_goal_pose_.pose.position.y - curr_pose.pose.position.y;
    correct_vec.vector.z = ur5_last_goal_pose_.pose.position.z - curr_pose.pose.position.z;
    ur5_last_goal_pose_ = curr_pose;
    ur5_last_goal_state_ = getCurrentRobotStateMsg();
    linearMove(correct_vec, false, true, false);

    // detach object
    ur5_.detachObject(object_id);
    ur5_last_goal_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // un-grasp ur5
    graspMove(-0.05, false, true, false);

    // move ur5 to prepare for push
    try_step = true;
    while (try_step && ros::ok()) {
        ROS_INFO("ENTE1");
        ur5_.setPoseReferenceFrame("ur5_ee_0");
        ur5_.setStartStateToCurrentState();
        // waypoints
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint;
        ur5_waypoint.position.x = 0;
        ur5_waypoint.position.y = 0;
        ur5_waypoint.position.z = 0;
        ur5_waypoint.orientation.x = 0;
        ur5_waypoint.orientation.y = 0;
        ur5_waypoint.orientation.z = 0;
        ur5_waypoint.orientation.w = 1;
        ur5_waypoints.push_back(ur5_waypoint);
        ur5_waypoint.position.x = ur5_waypoint.position.x;
        ur5_waypoint.position.y = ur5_waypoint.position.y + sceneManager.box_.dimensions[1]/2 + 0.05;
        ur5_waypoint.position.z = ur5_waypoint.position.z;
        ur5_waypoints.push_back(ur5_waypoint);

        moveit_msgs::RobotTrajectory ur5_trajectory;
        double fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, true);
        if (fraction < 0.9) {
            ROS_WARN("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_trajectory);
            moveit::planning_interface::MoveGroup::Plan ur5_plan;
            ur5_plan.trajectory_ = ur5_trajectory;
            execute(ur5_plan);
            ur5_.setPoseReferenceFrame("table_ground");
            try_step = false;
        }
    }
    try_step = true;
    while (try_step && ros::ok()) {
        ROS_INFO("ENTE2");
        ur5_.setPoseReferenceFrame("ur5_ee_0");
        ur5_.setStartStateToCurrentState();
        // waypoints
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint;
        ur5_waypoint.position.x = 0;
        ur5_waypoint.position.y = 0;
        ur5_waypoint.position.z = 0;
        ur5_waypoint.orientation.x = 0;
        ur5_waypoint.orientation.y = 0;
        ur5_waypoint.orientation.z = 0;
        ur5_waypoint.orientation.w = 1;
        ur5_waypoints.push_back(ur5_waypoint);

        ur5_rot = ur5_rot.Quaternion(ur5_waypoint.orientation.x, ur5_waypoint.orientation.y, ur5_waypoint.orientation.z, ur5_waypoint.orientation.w);
        ur5_rot.DoRotX(3.14/2);
        ur5_rot.GetQuaternion(ur5_waypoint.orientation.x, ur5_waypoint.orientation.y, ur5_waypoint.orientation.z, ur5_waypoint.orientation.w);
        ur5_waypoint.position.z = ur5_waypoint.position.z + 0.18;
        ur5_waypoints.push_back(ur5_waypoint);

        moveit_msgs::RobotTrajectory ur5_trajectory;
        double fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, true);
        if (fraction < 0.9) {
            ROS_WARN("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_trajectory);
            moveit::planning_interface::MoveGroup::Plan ur5_plan;
            ur5_plan.trajectory_ = ur5_trajectory;
            execute(ur5_plan);
            ur5_.setPoseReferenceFrame("table_ground");
            try_step = false;
        }
    }

    // move closer
    graspMove(0.05, false, true, false);

    // attach object again to ur5 and update State Msg
    ur5_.attachObject(object_id, ur5_.getEndEffectorLink());
    ur5_last_goal_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // push box into goal position  //TODO: method linear move with vector.
    geometry_msgs::Vector3Stamped directionStamped;
    directionStamped.header.frame_id = "/table_ground";
    // transform direction vector into frame "table_ground"
    KDL::Rotation shelf_table_rot;
    shelf_table_rot.DoRotZ(-3.14/4);
    KDL::Vector direction_vec;
    direction_vec.x(direction.x);
    direction_vec.y(direction.y);
    direction_vec.z(direction.z);
    direction_vec = shelf_table_rot * direction_vec;
    directionStamped.vector.x = direction_vec.x();
    directionStamped.vector.y = direction_vec.y();
    directionStamped.vector.z = direction_vec.z();
    linearMove(directionStamped, false, true, false);

    // detach object
    ur5_.detachObject(object_id);
    ur5_last_goal_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // moving away from object using the same vector in opposite direction
    directionStamped.vector.x = - directionStamped.vector.x;
    directionStamped.vector.y = - directionStamped.vector.y;
    directionStamped.vector.z = - directionStamped.vector.z;
    linearMove(directionStamped, false, true, true);

    return true;
}

bool DualArmRobot::moveObject(std::string object_id, geometry_msgs::PoseStamped ur5_pose, double scale) {
    ROS_INFO("Moving object with both arms");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    bool adapt_error;


    // plan ur5 plan
    allowedArmCollision(true, object_id);
    try_step = true;
    moveit::planning_interface::MoveGroup::Plan ur5_plan;
    while (try_step && ros::ok()) {
        adapt_error = false;
        ur5_.setPoseTarget(ur5_pose, ur5_.getEndEffectorLink());
        ur5_.setStartState(ur5_last_goal_state_);
        error = ur5_.plan(ur5_plan);
        // get both arms trajectory
        allowedArmCollision(true,object_id);
        if (adaptTrajectory(ur5_plan.trajectory_, arms_offset_, both_arms_trajectory, 0.5)) ROS_INFO("successfully calculated trajectory for both arms");
        else {
            ROS_WARN("Problem adapting trajectory");
            adapt_error = true;
        }
        if (error.val != 1 || adapt_error) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur5 into goal position");
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur5_plan.trajectory_, scale);
            try_step = false;
            allowedArmCollision(false, object_id);
        }
    }

    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    // visualize plan and execute
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 5);
    execute(both_arms_plan);

    allowedArmCollision(false,object_id);

    return true;
}

bool DualArmRobot::planMoveObject(std::string object_id, geometry_msgs::PoseStamped ur5_pose, double scale) {
    ROS_INFO("Moving object with both arms (planning only)");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;
    moveit_msgs::RobotTrajectory both_arms_trajectory;
    bool adapt_error;

    // evaluation
    ros::Time before_planning_ur5;
    ros::Time after_planning_ur5;
    ros::Duration planning_time_ur5;

    ros::Time before_planning_adaption;
    ros::Time after_planning_adaption;
    ros::Duration planning_time_adaption;

    ros::Time before_planning_loop;
    ros::Time after_planning_loop;
    ros::Duration planning_time_loop;

    unsigned int loops = 0;
    unsigned long int traj_steps;


    // plan ur5 plan
    allowedArmCollision(true, object_id);
    try_step = true;
    moveit::planning_interface::MoveGroup::Plan ur5_plan;
    before_planning_loop = ros::Time::now();
    allowedArmCollision(true,object_id);
    while (try_step && ros::ok()) {
        loops++;
        adapt_error = false;
        ur5_.setPoseTarget(ur5_pose, ur5_.getEndEffectorLink());
        ur5_.setStartState(ur5_last_goal_state_);

        before_planning_ur5 = ros::Time::now();
        error = ur5_.plan(ur5_plan);
        after_planning_ur5 = ros::Time::now();

        // get both arms trajectory
        before_planning_adaption = ros::Time::now();
        if (adaptTrajectory(ur5_plan.trajectory_, arms_offset_, both_arms_trajectory, 0.5)) ROS_INFO("successfully calculated trajectory for both arms");
        else {
            ROS_WARN("Problem adapting trajectory");
            adapt_error = true;
        }
        after_planning_adaption = ros::Time::now();
        if (error.val != 1 || adapt_error) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur5_plan.trajectory_, scale);
            try_step = false;
            allowedArmCollision(false, object_id);
        }
    }
    after_planning_loop = ros::Time::now();
    dual_arm_toolbox::TrajectoryProcessor::clean(both_arms_trajectory);

    // get plan from trajectory
    moveit::planning_interface::MoveGroup::Plan both_arms_plan;
    both_arms_plan.trajectory_ = both_arms_trajectory;
    both_arms_plan.start_state_.attached_collision_objects = getCurrentRobotStateMsg().attached_collision_objects;

    allowedArmCollision(false,object_id);
    //execute(both_arms_plan);

    // evaluation
    planning_time_ur5 = after_planning_ur5 - before_planning_ur5;
    traj_steps = both_arms_trajectory.joint_trajectory.points.size();
 //   std::cout << both_arms_trajectory.joint_trajectory.points.size();
    planning_time_adaption = after_planning_adaption - before_planning_adaption;
    planning_time_loop = after_planning_loop - before_planning_loop;

    ROS_INFO(":::::: VALUES EVALUATION ::::::");
    ROS_INFO("Planning Time for UR5 took: %li nsec", planning_time_ur5.toNSec());
    ROS_INFO("Planning Time for adaption took: %li nsec", planning_time_adaption.toNSec());
    ROS_INFO("Trajectory has %lu waypoints", traj_steps);
    ROS_INFO("Planning Time for loop took: %li nsec", planning_time_loop.toNSec());
    ROS_INFO("Planning loop count: %i steps", loops);

    // visualize plan
//    dual_arm_toolbox::TrajectoryProcessor::visualizePlan(both_arms_plan, 10);

    return true;
}

void DualArmRobot::allowedArmCollision(bool enable, std::string ur5_attachedObject) {
    // planning scene setup
    robot_model_loader::RobotModelLoader robot_model_loader(
            "robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planningScene(kinematic_model);
    collision_detection::AllowedCollisionMatrix acm = planningScene.getAllowedCollisionMatrix();
    moveit_msgs::PlanningScene planningSceneMsg;

    ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    // Robot links and attached object
    std::vector<std::string> ur5_links;
    ur5_links.push_back("ur5_ee_link");
    ur5_links.push_back("ur5_forearm_link");
    ur5_links.push_back("ur5_tool0");
    ur5_links.push_back("ur5_upper_arm_link");
    ur5_links.push_back("ur5_wrist_1_link");
    ur5_links.push_back("ur5_wrist_2_link");
    ur5_links.push_back("ur5_wrist_3_link");
    ur5_links.push_back("ur5_ee_0");
    ur5_links.push_back("ur5_ee_frame");
    ur5_links.push_back("ur5_ee_spring");
    ur5_links.push_back(ur5_attachedObject.c_str());

    std::vector<std::string> ur10_links;
    ur10_links.push_back("ur10_ee_link");
    ur10_links.push_back("ur10_forearm_link");
    ur10_links.push_back("ur10_tool0");
    ur10_links.push_back("ur10_upper_arm_link");
    ur10_links.push_back("ur10_wrist_1_link");
    ur10_links.push_back("ur10_wrist_2_link");
    ur10_links.push_back("ur10_wrist_3_link");
    ur10_links.push_back("ur10_ee_0");
    ur10_links.push_back("ur10_ee_frame");
    ur10_links.push_back("ur10_ee_spring");

    acm.setEntry(ur5_links, ur10_links, enable);  //Set an entry corresponding to all possible pairs between two sets of elements.

    // publish scene diff
    acm.getMessage(planningSceneMsg.allowed_collision_matrix);
    planningSceneMsg.is_diff = true;
    planning_scene_diff_publisher.publish(planningSceneMsg);
    ROS_INFO("Allowed collision between robot arms %s", enable?"ENABLED":"DISABLED");
    sleep(2);
}

bool DualArmRobot::linearMove(geometry_msgs::Vector3Stamped direction, bool avoid_collisions, bool use_ur5,
                              bool use_ur10) {

    bool try_step;
    double fraction;

    // ur5
    if (use_ur5) try_step = true;
    while (try_step && ros::ok()) {
        ur5_.setPoseReferenceFrame(direction.header.frame_id);
        ur5_.setStartState(ur5_last_goal_state_);
        // fk service client setup
        ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
        moveit_msgs::GetPositionFK fk_msg;
        fk_msg.request.header.frame_id = direction.header.frame_id;
        fk_msg.request.fk_link_names.push_back(ur5_.getEndEffectorLink());
        fk_msg.request.robot_state = ur5_last_goal_state_;
        fk_client.call(fk_msg.request, fk_msg.response);
        // get virtual pose from virtual state
        if (fk_msg.response.error_code.val != 1) {
            ROS_WARN("fk request error");
            return false;
        }
        geometry_msgs::PoseStamped ur5_start_pose = fk_msg.response.pose_stamped[0];

        // waypoints
        std::vector<geometry_msgs::Pose> ur5_waypoints;
        geometry_msgs::Pose ur5_waypoint = ur5_start_pose.pose;
        ur5_waypoints.push_back(ur5_waypoint);
        ur5_waypoint.position.x = ur5_waypoint.position.x + direction.vector.x;
        ur5_waypoint.position.y = ur5_waypoint.position.y + direction.vector.y;
        ur5_waypoint.position.z = ur5_waypoint.position.z + direction.vector.z;

        ur5_waypoints.push_back(ur5_waypoint);
        moveit_msgs::RobotTrajectory ur5_trajectory;
        fraction = ur5_.computeCartesianPath(ur5_waypoints, 0.001, 0.0, ur5_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("UR5 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        }
        else {
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_trajectory);
            moveit::planning_interface::MoveGroup::Plan ur5_plan;
            ur5_plan.trajectory_ = ur5_trajectory;
            execute(ur5_plan);
            try_step = false;
        }
    }

    // ur10
    if (use_ur10) try_step = true;
    while (try_step && ros::ok()) {
        ur10_.setStartStateToCurrentState();
        std::vector<geometry_msgs::Pose> ur10_waypoints;
        ur10_.setPoseReferenceFrame(direction.header.frame_id);
        geometry_msgs::Pose ur10_waypoint = ur10_.getCurrentPose(ur10_.getEndEffectorLink()).pose;
        ur10_waypoints.push_back(ur10_waypoint);

        // calculate waypoint
        ur10_waypoint.position.x = ur10_waypoint.position.x + direction.vector.x;
        ur10_waypoint.position.y = ur10_waypoint.position.y + direction.vector.y;
        ur10_waypoint.position.z = ur10_waypoint.position.z + direction.vector.z;

        ur10_waypoints.push_back(ur10_waypoint);
        moveit_msgs::RobotTrajectory ur10_trajectory;
        fraction = ur10_.computeCartesianPath(ur10_waypoints, 0.001, 0.0, ur10_trajectory, avoid_collisions);
        if (fraction < 0.9) {
            ROS_WARN("UR10 cartesian path. (%.2f%% achieved)", fraction * 100.0);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            moveit::planning_interface::MoveGroup::Plan ur10_plan;
            dual_arm_toolbox::TrajectoryProcessor::clean(ur10_trajectory);
            ur10_plan.trajectory_ = ur10_trajectory;
            execute(ur10_plan);
            try_step = false;
        }
    }
}

bool DualArmRobot::execute(moveit::planning_interface::MoveGroup::Plan plan) {
    ROS_INFO("executing trajectory");
#ifndef OFFLINE
    moveit::planning_interface::MoveGroup::Plan plan_ur5;
    moveit::planning_interface::MoveGroup::Plan plan_ur10;

    dual_arm_toolbox::TrajectoryProcessor::split(plan.trajectory_, plan_ur5.trajectory_,plan_ur10.trajectory_,"ur5","ur10");
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_ur5.trajectory_);
    dual_arm_toolbox::TrajectoryProcessor::clean(plan_ur10.trajectory_);

    bool success_ur10;
    bool success_ur5;

    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_ur5(ur5_controller_,"follow_joint_trajectory");
    moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_ur10(ur10_controller_,"follow_joint_trajectory");
    //moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle handle_ur10("ur10/ur10_vel_based_traj_admittance_controller","follow_joint_trajectory");

    // for both arms trajectories: because in planning each arm was not aware of the other there is a collision check before executing the trajectory
    /* TODO: put in again later
    if ((plan_ur5.trajectory_.joint_trajectory.joint_names.size() > 0) && (plan_ur10.trajectory_.joint_trajectory.joint_names.size() > 0)){
        // check trajectory for collisions
        robot_model_loader::RobotModelLoader robot_model_loader(
                "robot_description");
        robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
        planning_scene::PlanningScene planningScene(kinematic_model);
        bool isValid = planningScene.isPathValid(plan.start_state_,plan.trajectory_,"arms");
        if (!isValid){
            ROS_ERROR("Path is invalid. Execution aborted");
            return false;
        }
        else ROS_INFO("Checked path. Path is valid. Executing...");
    }*/

    if (plan_ur5.trajectory_.joint_trajectory.joint_names.size() > 0){
        ROS_INFO("Trajectory sent to ur5");
        success_ur5 = handle_ur5.sendTrajectory(plan_ur5.trajectory_);
    }

    if (plan_ur10.trajectory_.joint_trajectory.joint_names.size() > 0){
        ROS_INFO("Trajectory sent to ur10");
        success_ur10 = handle_ur10.sendTrajectory(plan_ur10.trajectory_);
    }

    if (plan_ur5.trajectory_.joint_trajectory.joint_names.size() > 0) success_ur5 = handle_ur5.waitForExecution();
    else success_ur5 = true;
    if (plan_ur10.trajectory_.joint_trajectory.joint_names.size() > 0) success_ur10 = handle_ur10.waitForExecution();
    else success_ur10 = true;
    sleep(0.5);  // to be sure robot is at goal position

    // update last goal ur5
    if (plan_ur5.trajectory_.joint_trajectory.joint_names.size()) {
        ur5_last_goal_state_.is_diff = true;
        ur5_last_goal_state_.joint_state.effort = plan_ur5.trajectory_.joint_trajectory.points[
                plan_ur5.trajectory_.joint_trajectory.points.size() - 1].effort;
        ur5_last_goal_state_.joint_state.header = plan_ur5.trajectory_.joint_trajectory.header;
        ur5_last_goal_state_.joint_state.name = plan_ur5.trajectory_.joint_trajectory.joint_names;
        ur5_last_goal_state_.joint_state.position = plan_ur5.trajectory_.joint_trajectory.points[
                plan_ur5.trajectory_.joint_trajectory.points.size() - 1].positions;
        ur5_last_goal_state_.joint_state.velocity = plan_ur5.trajectory_.joint_trajectory.points[
                plan_ur5.trajectory_.joint_trajectory.points.size() - 1].velocities;
    }

    // fk service client setup
    ros::ServiceClient fk_client = nh_.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    moveit_msgs::GetPositionFK fk_msg;
    fk_msg.request.header.frame_id = "table_ground";
    fk_msg.request.fk_link_names.push_back(ur5_.getEndEffectorLink());
    fk_msg.request.robot_state = ur5_last_goal_state_;
    fk_client.call(fk_msg.request, fk_msg.response);

    // get virtual pose from virtual state
    if (fk_msg.response.error_code.val != 1) {
        ROS_WARN("fk request error");
        return false;
    }
    ur5_last_goal_pose_ = fk_msg.response.pose_stamped[0];

    return success_ur5&&success_ur10;
#endif
#ifdef OFFLINE

    double error = arms_.execute(plan);
    sleep(2); // to be sure robot is at goal position
    ur5_last_goal_pose_ = ur5_.getCurrentPose(ur5_.getEndEffectorLink());
    ur5_last_goal_state_.is_diff = true;
    ur5_last_goal_state_.joint_state.position = ur5_.getCurrentJointValues();
    ur5_last_goal_state_.joint_state.name = ur5_.getJointNames();
    if (error == -1) return false;
    return true;
#endif
}

bool DualArmRobot::moveHome() {
    ROS_INFO("Moving arms into home position - both arms up");
    bool try_step;
    moveit::planning_interface::MoveItErrorCode error;

    // move ur5
    try_step = true;
    while (try_step && ros::ok()) {
        ur5_.setNamedTarget("ur5_up");
        moveit::planning_interface::MoveGroup::Plan ur5_plan;
        ur5_.setStartState(ur5_last_goal_state_);
        error = ur5_.plan(ur5_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur5 into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(ur5_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur5_plan.trajectory_, 0.4);
            execute(ur5_plan);
            try_step = false;
        }
    }

    // move ur10
    try_step = true;
    while (try_step && ros::ok()) {
        ur10_.setNamedTarget("ur10_up");
        moveit::planning_interface::MoveGroup::Plan ur10_plan;
        error = ur10_.plan(ur10_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving ur10 into grasp position");
            dual_arm_toolbox::TrajectoryProcessor::clean(ur10_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(ur10_plan.trajectory_, 0.4);
            execute(ur10_plan);
            try_step = false;
        }
    }

    // move both at simultaneously
    /*
    try_step = true;
    while (try_step && ros::ok()) {
        arms_.setNamedTarget("arms_up");
        moveit::planning_interface::MoveGroup::Plan arms_plan;
        arms_.setStartState(ur5_last_goal_state_);
        error = arms_.plan(arms_plan);
        if (error.val != 1) {
            ROS_WARN("MoveIt!Error Code: %i", error.val);
            try_step = try_again_question();
            if (!try_step) return false;
        } else {
            ROS_INFO("Moving arms into home position");
            dual_arm_toolbox::TrajectoryProcessor::clean(arms_plan.trajectory_);
            dual_arm_toolbox::TrajectoryProcessor::scaleTrajectorySpeed(arms_plan.trajectory_, 0.4);
            execute(arms_plan);
            try_step = false;
        }
    }
    */

    return true;
}
