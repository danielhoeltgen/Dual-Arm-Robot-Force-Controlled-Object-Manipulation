#include <ros/ros.h>
//#include <time.h>

class Stopwatch{
protected:
public:
    Stopwatch();
    void restart();
    ros::Duration elapsed();
private:
    ros::Time start_;
};
