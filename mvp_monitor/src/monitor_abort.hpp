#pragma once

#include <unordered_map>
#include <algorithm>

#include <ros/ros.h>
#include <mvp_msgs/GetState.h>
#include <mvp_msgs/GetStates.h>
#include <mvp_msgs/ChangeState.h>
#include <mvp_msgs/HelmState.h>

#include "default.hpp"

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

    std::string srv_get_state_;
    
    std::string srv_get_states_;

    std::string srv_change_state_;

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
     * verify all the states are able to change
     * 
    */
    bool verifyAbortAction();

    /**
     * verify all the ros service are available 
     * 
    */
    void verifySrv();

    void setupRos();
    
    void initialize();

public:
    MonitorAbort(          
        const ros::NodeHandle &nh,
        const ros::NodeHandle &nh_private);

};
