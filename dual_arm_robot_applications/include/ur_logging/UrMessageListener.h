#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

class UR_Message_Listener{  //handles callbacks and saves last received messages
protected:
    ros::NodeHandle nh_;
    ros::Subscriber wrench_sub_;
    ros::Subscriber speed_traj_sub_;
    ros::Subscriber state_sub_;
public:
    UR_Message_Listener(ros::NodeHandle& nh, std::string ur_namespace);

    std::string ur_namespace_;
    trajectory_msgs::JointTrajectory last_speed_traj_msg_;
    geometry_msgs::WrenchStamped last_wrench_msg_;
    sensor_msgs::JointState last_state_msg_;
    std::string ur_prefix_;
private:
    void wrenchCallback(const geometry_msgs::WrenchStamped::Ptr& msg);
    void speed_trajCallback(const trajectory_msgs::JointTrajectory::Ptr& msg);
    void stateCallback(const sensor_msgs::JointState::Ptr& msg);
};
