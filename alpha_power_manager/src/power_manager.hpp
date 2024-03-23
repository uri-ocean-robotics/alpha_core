/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "memory"
#include <string>
#include <vector>
#include <functional>


class PowerManager
{
    private:
        ros::NodeHandlePtr m_nh;
        ros::NodeHandlePtr m_pnh;

        std::vector<std::string> m_device_name;
        std::vector<int> m_gpio_name;
        int m_gpio_count;

        struct gpio_t
        {
            std::string device_name;
            std::string gpio_name;
            std::string service_name; 
            ros::ServiceServer m_set_gpio;
        };
        std::vector<gpio_t> gpio_vector;

        ros::ServiceServer m_get_p_state;

        bool f_cb_srv_set_power(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res, std::string gpio_name);

        bool f_cb_srv_get_state(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res);
        bool f_set_gpio_value(std::string value, std::string gpio_name);
        bool f_initialize_gpio();

    public:
        PowerManager();
};