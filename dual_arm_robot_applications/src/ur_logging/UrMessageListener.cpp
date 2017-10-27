# include "ur_logging/UrMessageListener.h"


//UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
UR_Message_Listener::UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace) : nh_(nh), ur_namespace_(ur_namespace){
    wrench_sub_ = nh_.subscribe(ur_namespace+"/tcp_wrench", 1, &UR_Message_Listener::wrenchCallback, this);
    speed_traj_sub_ = nh_.subscribe(ur_namespace+"/ur_driver/joint_speed", 1, &UR_Message_Listener::speed_trajCallback, this);
    state_sub_ = nh_.subscribe("/joint_states", 1, &UR_Message_Listener::stateCallback, this);

    if (ur_namespace_.size() > 0){
        ur_prefix_ = ur_namespace_+"_";
    }
}

void UR_Message_Listener::wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg){
    last_wrench_msg_ = *msg;
}


void UR_Message_Listener::speed_trajCallback(const trajectory_msgs::JointTrajectory::Ptr& msg){
    last_speed_traj_msg_ = *msg;
}


void UR_Message_Listener::stateCallback(const sensor_msgs::JointState::Ptr& msg){
    std::string name = (*msg).name[0];
    if (name.compare(0,ur_prefix_.size(), ur_prefix_) == 0){    // "joint_states" receives messages of all robots. This is seperating the searched one from the others.
        last_state_msg_ = *msg;
    }
}
