#pragma once

#include <ros/ros.h>

class MonitorAbort {
private:
    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    double monitor_rate_;

    ros::Timer timer_;

    void loadParameters();

    void timerCallback(const ros::TimerEvent& event);

public:
    MonitorAbort(          
        const ros::NodeHandle &nh,
        const ros::NodeHandle &nh_private);

};
