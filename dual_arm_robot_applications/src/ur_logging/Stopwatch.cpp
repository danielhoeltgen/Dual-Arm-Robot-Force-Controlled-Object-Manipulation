#include "ur_logging/Stopwatch.h"

Stopwatch::Stopwatch(){
    start_ = ros::Time::now();
}

void Stopwatch::restart(){
    start_ = ros::Time::now();
}

ros::Duration Stopwatch::elapsed(){
    return (ros::Time::now() - start_);
}
