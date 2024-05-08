#include "monitor_abort.hpp"

MonitorAbort::MonitorAbort(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &nh_private) 
    : nh_(nh), pnh_(nh_private) 
{
    loadParameters();


    // setup timer callback to time counting

    timer_ = nh_.createTimer(ros::Duration(1.0 / monitor_rate_), &MonitorAbort::timerCallback, this);
}

void MonitorAbort::loadParameters()
{
    // read some parameters
    pnh_.param<double>("monitor_rate_", monitor_rate_, 1.0);
}

void MonitorAbort::timerCallback(const ros::TimerEvent& event)
{
    ROS_INFO("hi from MVP_Monitor timercallback");
}
