/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#include "power_manager.hpp"
#include <iostream> 

PowerManager::PowerManger()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->param<std::string>("p1_device_names", m_p1_device_name, "");
    m_pnh->param<std::string>("p2_device_names", m_p2_device_name, "");
    m_pnh->param<std::string>("p3_device_names", m_p3_device_name, "");


    m_set_p1 = m_nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
    (
        "Set_Port1_Power",
        std::bind(
            &PowerManager::f_cb_srv_set_p1,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    //set the gpio direction
    system("echo 9 > /sys/class/gpio/export");
    system("echo 12 > /sys/class/gpio/export");
    system("echo 13 > /sys/class/gpio/export");
    system("echo out > /sys/class/gpio/gpio9/direction");
    system("echo out > /sys/class/gpio/gpio12/direction");
    system("echo out > /sys/class/gpio/gpio13/direction");
    system("echo 0 > /sys/class/gpio/gpio9/value");
    system("echo 0 > /sys/class/gpio/gpio12/value");
    system("echo 0 > /sys/class/gpio/gpio13/value"); 
}

void PowerManager::f_cb_srv_set_p1(
        std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    if(req.data)
    {
        system("echo 1 > /sys/class/gpio/gpio9/value");
        res.success = 1;
        res.message = getchar();
        printf("Port 1 power enabled\r\n");
    }
    else
    {
        system("echo 0 > /sys/class/gpio/gpio9/value");
        res.success = 1;
        res.message = getchar();
        printf("Port 1 power disabled\r\n");
    }
}


void PowerManager::f_cb_srv_get_state(
                        std_srvs::Trigger::Request &req,
                        std:srvs::Trigger::Response &res)
{
    char v[3];
    std::string msg;
    v[0] = system("cat /sys/class/gpio/gpio9/value");
    v[1] = system("cat /sys/class/gpio/gpio9/value");
    v[2] = system("cat /sys/class/gpio/gpio9/value");

    msg = "Port 1 state =" + v[0] + "| Device Name = " + m_p1_device_name + "\r\n"
        + "Port 2 state =" + v[1] + "| Device Name = " + m_p2_device_name + "\r\n"
        + "Port 3 state =" + v[2] + "| Device Name = " + m_p3_device_name + "\r\n";

    res.success = 1;
    res.message = msg;
    printf("Power Manage Info:\r\n");
    printf("%s", msg.c_str());
}