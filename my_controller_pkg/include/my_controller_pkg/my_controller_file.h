namespace my_controller_ns{

class MyControllerClass
{
private:
  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
} 
