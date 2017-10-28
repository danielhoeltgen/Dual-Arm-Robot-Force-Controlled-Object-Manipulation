//#include "my_controller_pkg/my_controller_file.h"

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>

/*
#include <joint_admittance_trajectory_controller/hardware_interface_adapter.h>
#include <joint_admittance_trajectory_controller/init_joint_trajectory.h>
#include <joint_admittance_trajectory_controller/joint_admittance_trajectory_controller.h>
#include <joint_admittance_trajectory_controller/joint_trajectory_controller_impl.h>
#include <joint_admittance_trajectory_controller/joint_trajectory_msg_utils.h>
#include <joint_admittance_trajectory_controller/joint_trajectory_segment.h>
#include <joint_admittance_trajectory_controller/tolerances.h>
*/
/*
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
*/

namespace controller_ns{

class CartController : public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::VelocityJointInterface>
{
    /*
    void update(const ros::Time &time, const ros::Duration &period){
        // Get currently followed trajectory
        TrajectoryPtr curr_traj_ptr;
        curr_trajectory_box_.get(curr_traj_ptr);
        Trajectory& curr_traj = *curr_traj_ptr;

        // Update time data
        TimeData time_data;
        time_data.time   = time;                                     // Cache current time
        time_data.period = period;                                   // Cache current control period
        time_data.uptime = time_data_.readFromRT()->uptime + period; // Update controller uptime
        time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

        // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
        // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
        // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
        // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
        // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
        // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
        // next control cycle, leaving the current cycle without a valid trajectory.

        // Update desired state: sample trajectory at current time
        typename Trajectory::const_iterator segment_it = sample(curr_traj, time_data.uptime.toSec(), desired_state_);
        if (curr_traj.end() == segment_it)
        {
          // Non-realtime safe, but should never happen under normal operation
          ROS_ERROR_NAMED(name_,
                          "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
          return;
        }

        // Update current state and state error
        for (unsigned int i = 0; i < joints_.size(); ++i)
        {
          current_state_.position[i] = joints_[i].getPosition();
          current_state_.velocity[i] = joints_[i].getVelocity();
          // There's no acceleration data available in a joint handle

          state_error_.position[i] = desired_state_.position[i] - current_state_.position[i];
          state_error_.velocity[i] = desired_state_.velocity[i] - current_state_.velocity[i];
          state_error_.acceleration[i] = 0.0;

          // Check tolerances if segment corresponds to currently active action goal
          const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
          if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
          {
            // Check tolerances
            if (time_data.uptime.toSec() < segment_it->endTime())
            {
              // Currently executing a segment: check path tolerances
              checkPathTolerances(state_error_,
                                  *segment_it);
            }
            else if (segment_it == --curr_traj.end())
            {
              if (verbose_)
                ROS_DEBUG_STREAM_THROTTLE_NAMED(1,name_,"Finished executing last segement, checking goal tolerances");

              // Finished executing the LAST segment: check goal tolerances
              checkGoalTolerances(state_error_,
                                   *segment_it);
            }
          }

          // Hardware interface adapter: Generate and send commands
          hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
                                          desired_state_, state_error_);

          // Set action feedback
          if (rt_active_goal_)
          {
            rt_active_goal_->preallocated_feedback_->header.stamp          = time_data_.readFromRT()->time;
            rt_active_goal_->preallocated_feedback_->desired.positions     = desired_state_.position;
            rt_active_goal_->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
            rt_active_goal_->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
            rt_active_goal_->preallocated_feedback_->actual.positions      = current_state_.position;
            rt_active_goal_->preallocated_feedback_->actual.velocities     = current_state_.velocity;
            rt_active_goal_->preallocated_feedback_->error.positions       = state_error_.position;
            rt_active_goal_->preallocated_feedback_->error.velocities      = state_error_.velocity;
            rt_active_goal_->setFeedback( rt_active_goal_->preallocated_feedback_ );
          }

          // Publish state
          publishState(time_data.uptime);
        }
    }
private:
    struct TimeData
    {
      TimeData() : time(0.0), period(0.0), uptime(0.0) {}

      ros::Time     time;   ///< Time of last update cycle
      ros::Duration period; ///< Period of last update cycle
      ros::Time     uptime; ///< Controller uptime. Set to zero at every restart.
    };

    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
    typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
    typedef ActionServer::GoalHandle                                                            GoalHandle;
    typedef realtime_tools::RealtimeServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RealtimeGoalHandle;
    typedef boost::shared_ptr<RealtimeGoalHandle>                                               RealtimeGoalHandlePtr;
    typedef trajectory_msgs::JointTrajectory::ConstPtr                                          JointTrajectoryConstPtr;
    typedef realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>     StatePublisher;
    typedef boost::scoped_ptr<StatePublisher>                                                   StatePublisherPtr;

    typedef JointTrajectorySegment<SegmentImpl> Segment;
    typedef std::vector<Segment> Trajectory;
    typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
    typedef realtime_tools::RealtimeBox<TrajectoryPtr> TrajectoryBox;
    typedef typename Segment::Scalar Scalar;

    typedef HardwareInterfaceAdapter<HardwareInterface, typename Segment::State> HwIfaceAdapter;
    typedef typename HardwareInterface::ResourceHandleType JointHandle;
     */
};



PLUGINLIB_EXPORT_CLASS(controller_ns::CartController, controller_interface::ControllerBase)

}//namespace
