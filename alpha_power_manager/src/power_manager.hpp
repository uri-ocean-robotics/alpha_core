/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#pragma once

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "memory"


class PowerManager
{
    private:
        ros::NodeHandlePtr m_nh;
        ros::NodeHandlePtr m_pnh;

        std::string m_p1_device_name;
        std::string m_p2_device_name;
        std::string m_p3_device_name;
        std::string m_p1_io;
        std::string m_p2_io;
        std::string m_p3_io;


        ros::ServiceServer m_set_p1;
        ros::ServiceServer m_set_p2;
        ros::ServiceServer m_set_p3;
        ros::ServiceServer m_get_p_state;


        bool f_cb_srv_set_p1(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res);

        // void f_cb_srv_set_p2(std_srvs::SetBool::Request &req,
        //                    std_srvs::SetBool::Response &res);

        // void f_cb_srv_set_p3(std_srvs::SetBool::Request &req,
        //                    std_srvs::SetBool::Response &res);

        bool f_cb_srv_get_state(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);

    public:
        PowerManager();
};