#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <time.h>

#include <iomanip>
#include <locale>

#include "ur_logging/Stopwatch.h"
#include "ur_logging/UrMessageListener.h"

class UR_Logger{    //Logs all important messages from ur. It can handle several robots at a time
protected:
    ros::NodeHandle nh_;
    std::vector<UR_Message_Listener*> ur_listeners_;

public:
    UR_Logger(ros::NodeHandle& nh, std::vector<std::string> ur_namespaces);
    ~UR_Logger();

    Stopwatch stopwatch_;
    std::string logfile_name_;
    char delimiter_;

    void generate_logfile_name();       //automatically generate a name
    void start(int log_rate);     //log_rate=[Hz]
    void stop();
    std::string headline(UR_Message_Listener ur_listener);	//return a headline
    std::string data_line(UR_Message_Listener ur_listener);	//return formattet data
private:
    ros::Timer timer_;
    std::ofstream logfile_;
    ros::Time log_start_time_;

    void logCallback(const ros::TimerEvent&);
};
