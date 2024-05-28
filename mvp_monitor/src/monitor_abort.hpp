#pragma once

#include <unordered_map>
#include <algorithm>

#include <ros/ros.h>
#include <mvp_msgs/GetState.h>
#include <mvp_msgs/GetStates.h>
#include <mvp_msgs/ChangeState.h>
#include <mvp_msgs/HelmState.h>

#include "default.hpp"

#define CONST_STRING static constexpr const char *

// CONST_STRING CONF_ABORT = "abort_action";
// CONST_STRING CONF_ABORT_STATE = "state";
// CONST_STRING CONF_ABORT_TIMEOUT = "timeout";
// CONST_STRING CONF_ABORT_TRANSITION = "transition";

struct AbortAction {
    double timeout;
    std::string transition;
};

//! TODO: right now the monitoring process not require real-time performance
//        we should use callbacl function for the state changes if we need real-time requirement

class MonitorAbort {
private:
    // ros related

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::ServiceClient clinet_get_state_;

    ros::ServiceClient clinet_get_states_;

    ros::ServiceClient clinet_change_state_;

    ros::Timer timer_;

    // global variables

    double time_count_;

    mvp_msgs::HelmState curr_state_;

    mvp_msgs::HelmState last_state_;

    // parameters

    double monitor_rate_;

    std::string name_space_;

    std::string topic_get_state_;
    
    std::string topic_get_states_;

    std::string topic_change_state_;

    std::unordered_map<std::string, AbortAction> abort_action_;

    // functions 

    void loadParameters();

    void timerCallback(const ros::TimerEvent& event);

    /**
     * use ros srv to get MVP states and check if received state is inside our monitoring list
     * 
    */
    bool getState();

    /**
     * use ros srv to get all MVP states
     * 
    */
    bool getStates();

    void initialize();

public:
    MonitorAbort(          
        const ros::NodeHandle &nh,
        const ros::NodeHandle &nh_private);

};
