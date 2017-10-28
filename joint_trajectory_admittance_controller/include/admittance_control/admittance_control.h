//
// Created by Daniel Höltgen on 07.10.16.
//

// ROS
#include <ros/ros.h>

// Standard
#include <stdio.h>
#include <iostream>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>   //Newton-Raphson //takes joint limits into account
#include <kdl/chainiksolverpos_nr.hpp>  //Newton-Raphson

// Wrench
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

// PID
#include <control_toolbox/pid.h>

class AdmittanceControl{
protected:
    ros::NodeHandle nh_;

    // TODO: delete
    int count_;

    // KDL
    KDL::Chain kdl_chain_;

    // wrench
    ros::Subscriber wrench_sub_;
    void wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg);
    KDL::Vector force_;

    // Parameters
    double max_vel_;
    double max_shift_;
    double contact_F;

    // runtime variables
    double d_;  // distance
    KDL::JntArray q_last_;  // last joint state
    std::vector<double> new_start_position_;

    // Control, PID
    double wrench_tolerance_;
    control_toolbox::Pid pid_controller_;
public:
    AdmittanceControl();
    ~AdmittanceControl();
    void init(ros::NodeHandle &nh);
    void starting();
    void update_admittance_state(std::vector<double>& position, const ros::Duration& period);
    void set_shift(double d);
private:
};


AdmittanceControl::AdmittanceControl(){
}

AdmittanceControl::~AdmittanceControl(){
}

void AdmittanceControl::wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg){  // wrench in coordinate system of base
    geometry_msgs::WrenchStamped wrench_msg = *msg;
    force_.x(wrench_msg.wrench.force.x);
    force_.y(wrench_msg.wrench.force.y);
    force_.z(wrench_msg.wrench.force.z);
}

void AdmittanceControl::init(ros::NodeHandle &nh){
    // TODO: delete
    count_ = 0;

    nh_ = nh;
    // init variables
    d_ = 0.0;
    new_start_position_.clear();

    // KDL
    KDL::Tree kdl_tree;
    std::string robot_desc_string;
    std::string root_name;
    std::string tip_name;

    nh.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
        ROS_ERROR("Failed to construct kdl tree in namespace: %s", nh.getNamespace().c_str());    }
    else ROS_INFO("got kdl tree");

    nh.param("root_name", root_name, std::string());
    nh.param("tip_name", tip_name, std::string());

    std::cout << "root name, tip name" << root_name << "   " << tip_name << std::endl;
    std::cout << "Namespace: " << nh.getNamespace() << std::endl;

    if (!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
        ROS_ERROR("Failed to construct kdl chain with root: %s and tip: %s", root_name.c_str(), tip_name.c_str());
    else ROS_INFO("got kdl chain");

    // Settings
    nh.param("max_velocity", max_vel_, double());
    nh.param("max_shift", max_shift_, double());
    nh.param("contact_force", contact_F, double());
    contact_F = -std::abs(contact_F);   // must be a negative value

    // PID
    if (!pid_controller_.init(ros::NodeHandle(nh_, "admittance_pid"))){
        ROS_ERROR("Admittance Control: could not construct PID controller");
    }
    pid_controller_.reset();

    // wrench
    std::string wrench_topic;
    nh.param("wrench_tolerance", wrench_tolerance_, double());
    nh.param("wrench_topic", wrench_topic, std::string());
    wrench_sub_ = nh_.subscribe(wrench_topic, 1, &AdmittanceControl::wrenchCallback, this);
}

void AdmittanceControl::starting() {
    ROS_INFO("Starting Admittance Controller");
    set_shift(0);
    q_last_.resize(0);
}

void AdmittanceControl::update_admittance_state(std::vector<double>& position, const ros::Duration& period) {
    // Solver
    KDL::ChainIkSolverVel_pinv ikSolverVel(kdl_chain_);
    KDL::ChainFkSolverPos_recursive fkSolverPos(kdl_chain_);
    KDL::ChainIkSolverPos_NR ikSolverPosNR(kdl_chain_, fkSolverPos, ikSolverVel, 100, 1e-6);

    // Joint Array in
    unsigned int nj = kdl_chain_.getNrOfJoints();

    KDL::JntArray q_in(nj);

    for (int i = 0; i < nj; i++) {
        q_in(i, 0) = position[i];
    }

    // forward kinematics
    KDL::Frame p_eef;
    int fk_feedback = fkSolverPos.JntToCart(q_in, p_eef);    // segmentNr = -1
    if (fk_feedback < 0) {
        ROS_WARN("Admittance Control: Problem solving forward kinematics. Error: %s",
                 fkSolverPos.strError(fk_feedback));
        return;
    }

    // wrench at eef frame
    KDL::Vector wrench_eef;    // wrench in destination Frame eef
    ros::spinOnce();            // receive wrench msg
    wrench_eef = p_eef.M.Inverse() * force_;  // Rotate to get Force in coordinate system of the eef

    // PID
    if (std::abs(wrench_eef.z()-contact_F) > wrench_tolerance_) {
        if (((wrench_eef.z() < contact_F) && (d_ > -max_shift_)) ||
            ((wrench_eef.z() > contact_F) && (d_ < max_shift_))) {
            double pid_vel = pid_controller_.computeCommand(wrench_eef.z() - contact_F, period);
            if (std::abs(pid_vel) < max_vel_) d_ = d_ + pid_vel * period.toSec();
            else d_ = d_ + (std::abs(pid_vel) / pid_vel) * max_vel_ * period.toSec();
        }
    }

    // distance is transformed to be along z-axis of eef
    KDL::Vector vec_d;  // distance vector
    vec_d.x(0);
    vec_d.y(0);
    vec_d.z(d_);
    vec_d = p_eef.M*vec_d;     // Rotate distance vector

    // target frame
    KDL::Frame p_target = p_eef;
    p_target.p = p_eef.p + vec_d;  // Add distance Vector to eef Frame

    // target joint state
    KDL::JntArray q_dest(nj);

    // inverse kinematics
    KDL::JntArray q_init;
    if (q_last_.rows() == nj) q_init = q_last_;
    else q_init = q_in;     // use input state only first time. The last state should be closer to the solution in other cases
    int ik_feedback = ikSolverPosNR.CartToJnt(q_init, p_target, q_dest);
    if (ik_feedback < 0) {
        ROS_WARN("Admittance Control: Problem solving inverse kinematics. Error: %s", ikSolverPosNR.strError(ik_feedback));
        q_dest = q_init;
    }

    // check for jerk motion
    for (unsigned int i = 0; i < nj; i++){
        if (std::abs(q_dest(i,0)-q_init(i,0)) > 0.3) {
            ROS_WARN("Admittance Control: Jerk motion detected! Stop immediately!");
            q_dest = q_in;
            return;
        }
    }
    q_last_ = q_dest;

    // write position
    for (unsigned int i = 0; i < nj; i++){
        position[i] = q_dest(i,0);
    }
    // only position information necessary, because only the position will be forwarded to hardware_interface::setCommand()
}

void AdmittanceControl::set_shift(double d){
    d_ = d;
    ROS_WARN("Set shift to %f", d);
}