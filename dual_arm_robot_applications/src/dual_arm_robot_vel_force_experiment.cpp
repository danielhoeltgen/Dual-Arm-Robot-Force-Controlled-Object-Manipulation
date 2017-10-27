//
// Created by Daniel Höltgen.
//

#include <moveit/move_group_interface/move_group.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/SwitchControllerRequest.h>
#include <controller_manager_msgs/SwitchControllerResponse.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ur_logging/UrLogger.h"

/* TODO
 * vel ansteigen, auf null fallen, rückwärts bewegen
 */

// m/s ; m
void run_experiment(ros::NodeHandle &nh, ros::Publisher &ur10_speed_pub, moveit::planning_interface::MoveGroup &ur10, double velocity, double moving_distance){
    // create ur_logger. Use this namespace
    std::vector<std::string> ur_namespaces;
    ur_namespaces.push_back("");
    UR_Logger ur_logger(nh, ur_namespaces);

    // stop controller
    ros::ServiceClient ur10_srv_switch_controller = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv_req;
    srv_req.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    srv_req.request.stop_controllers.push_back("vel_based_pos_traj_controller");
    bool success = ur10_srv_switch_controller.call(srv_req);
    ROS_INFO("Stopping controller %s",success?"SUCCEDED":"FAILED");
    srv_req.request.stop_controllers.clear();

    geometry_msgs::PoseStamped current_pose = ur10.getCurrentPose();
    std::cout << "x: " << current_pose.pose.position.x << "\ty: " << current_pose.pose.position.y << "\tz:  " << current_pose.pose.position.z << std::endl;
    double radius = sqrt(current_pose.pose.position.x*current_pose.pose.position.x + current_pose.pose.position.y*current_pose.pose.position.y);
    std::cout << "RADIUS: " << radius << std::endl;
    double omega = velocity/radius;
    double moving_time = moving_distance/velocity; //move 3cm, obstacle is around 1-2 cm away.

    // Output Message Speed = const
    trajectory_msgs::JointTrajectory joint_traj; //containing speed command
    trajectory_msgs::JointTrajectoryPoint traj_point;
    traj_point.velocities.assign(6,0);
    traj_point.velocities[0] = -omega;
    joint_traj.points.push_back(traj_point);

    // start logging
    ur_logger.start(50);

    ROS_INFO("Publishing velocity commands to ur10 at 125Hz");

    // publish messages
    ros::Rate loop_rate(125);   // velocity-message publish rate

    Stopwatch stopwatch;

    while (ros::ok() && (stopwatch.elapsed().toSec()<moving_time))
    {
        joint_traj.header.stamp = ros::Time::now();
        ur10_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("setting vel to 0");
    joint_traj.points.pop_back();
    traj_point.velocities[0] = 0.0;
    joint_traj.points.push_back(traj_point);

    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec()<1.5))
    {
        joint_traj.header.stamp = ros::Time::now();
        ur10_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    joint_traj.points.pop_back();
    traj_point.velocities[0] = omega;
    joint_traj.points.push_back(traj_point);

    ROS_INFO("moving back");
    stopwatch.restart();

    while (ros::ok() && (stopwatch.elapsed().toSec()<moving_time))
    {
        joint_traj.header.stamp = ros::Time::now();
        ur10_speed_pub.publish(joint_traj);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Stopped publishing");

    // stop logging
    ur_logger.stop();

    // restart controller
    srv_req.request.BEST_EFFORT;
    srv_req.request.start_controllers.push_back("vel_based_pos_traj_controller");
    success = ur10_srv_switch_controller.call(srv_req);
    ROS_INFO("Starting controller %s",success?"SUCCEDED":"FAILED");
    srv_req.request.start_controllers.clear();
}


int main(int argc, char **argv)
{
    // ROS Setup
    ros::init(argc, argv, "ur_const_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher ur10_speed_pub = nh.advertise<trajectory_msgs::JointTrajectory>("ur_driver/joint_speed", 1);
    ros::AsyncSpinner asyncSpinner(2);
    asyncSpinner.start();

    // MoveGroup
    moveit::planning_interface::MoveGroup ur10("manipulator");
    moveit::planning_interface::MoveGroup::Plan plan;
    ur10.setPlanningTime(30);

    ROS_WARN("robot is moving without collision checking. BE CAREFUL!");
    ROS_INFO("waiting 10 Seconds. Press Ctrl-C if Robot is in the wrong start position");
    ros::Duration(10).sleep();

    //long distance position
    /*
    ur10.setJointValueTarget("elbow_joint", -1.366541959239651);
    ur10.setJointValueTarget("shoulder_lift_joint", -2.573810648739345);
    ur10.setJointValueTarget("shoulder_pan_joint", 0.5943102022167164);
    ur10.setJointValueTarget("wrist_1_joint", -0.7533539281232803);
    ur10.setJointValueTarget("wrist_2_joint", -0.0);
    ur10.setJointValueTarget("wrist_3_joint", 0.00015758105264953662);*/

    // short distance postion
    ur10.setJointValueTarget("elbow_joint", -1.9646600642315206);
    ur10.setJointValueTarget("shoulder_lift_joint", -2.2494529549924893);
    ur10.setJointValueTarget("shoulder_pan_joint", -1.1800545151007107);
    ur10.setJointValueTarget("wrist_1_joint", -0.41466027950402257);
    ur10.setJointValueTarget("wrist_2_joint", -0.0);
    ur10.setJointValueTarget("wrist_3_joint", 0.00300112795031922);

    //ur10.setJointValueTarget(ur10JointTarget);
    ur10.plan(plan);
    ROS_WARN("visualizing plan. STRG+C to interrupt.");
    sleep(4);
    ur10.execute(plan);
    sleep(3);

    // ::::::: Run Experiments :::::::
/*
    ROS_INFO("::: Experiment 1 ::: vel = 0.02m/s; s = 0.03m");
    run_experiment(nh, ur10_speed_pub, ur10, 0.02, 0.03);

    ROS_INFO("moveing to start");
    ur10.plan(plan);
    ur10.execute(plan);

    ROS_INFO("::: Experiment 2 ::: vel = 0.01m/s; s = 0.03m");
    run_experiment(nh, ur10_speed_pub, ur10, 0.01, 0.03);

    ROS_INFO("moveing to start");
    ur10.plan(plan);
    ur10.execute(plan);

    ROS_INFO("::: Experiment 3 ::: vel = 0.005m/s; s = 0.03m");
    run_experiment(nh, ur10_speed_pub, ur10, 0.005, 0.03);

    ROS_INFO("moveing to start");
    ur10.plan(plan);
    ur10.execute(plan);
*/
    ROS_INFO("::: Experiment 4 ::: vel = 0.001m/s; s = 0.03m");
    run_experiment(nh, ur10_speed_pub, ur10, 0.001, 0.03);

    ROS_INFO("moveing to start");
    ur10.plan(plan);
    ur10.execute(plan);


    ROS_INFO("finished. shutting down.");

    ros::shutdown();
    return 0;
}
