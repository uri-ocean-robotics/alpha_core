/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#include "power_manager.hpp"
#include <iostream> 
#include <string>
#include <sys/stat.h>

PowerManager::PowerManager()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->param<std::string>("p1_device_names", m_p1_device_name, "");
    m_pnh->param<std::string>("p2_device_names", m_p2_device_name, "");
    m_pnh->param<std::string>("p3_device_names", m_p3_device_name, "");
    m_pnh->param<std::string>("p1_gpio", m_p1_io, "9");
    m_pnh->param<std::string>("p2_gpio", m_p2_io, "12");
    m_pnh->param<std::string>("p3_gpio", m_p3_io, "13");


    m_set_p1 = m_nh->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
    (
        "set_port1_power",
        std::bind(
            &PowerManager::f_cb_srv_set_p1,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );


    m_get_p_state = m_nh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>
    (
        "get_power_port_status",
        std::bind(
            &PowerManager::f_cb_srv_get_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    // system("dir");
    // //enable GPIO, set direct, set value to 0 initially//
    std::string cmd;
    struct stat sb;
    cmd = "/sys/class/gpio/gpio" + m_p1_io;
    if( stat(cmd.c_str(), &sb) != 0 )
    {
        cmd = "echo " + m_p1_io + " > /sys/class/gpio/export";
        system(cmd.c_str());
    }
    // cmd = "echo " + m_p2_io + " > /sys/class/gpio/export";
    // system(cmd.c_str());
    // cmd = "echo " + m_p3_io + " > /sys/class/gpio/export";
    // system(cmd.c_str());
    
    cmd = "echo out > /sys/class/gpio/gpio" + m_p1_io + "/direction";
    printf("cmd = %s\r\n", cmd.c_str());
    system(cmd.c_str());
    // cmd = "echo out > /sys/class/gpio/gpio" + m_p2_io + "/direction";
    // system(cmd.c_str());
    // cmd = "echo out > /sys/class/gpio/gpio" + m_p3_io + "/direction";
    // system(cmd.c_str());

    cmd = "echo 0 > /sys/class/gpio/gpio" + m_p1_io + "/value";    
    printf("cmd = %s\r\n", cmd.c_str());
    system(cmd.c_str());
    // cmd = "echo 0 > /sys/class/gpio/gpio" + m_p2_io + "/value";    
    // system(cmd.c_str());
    // cmd = "echo 0 > /sys/class/gpio/gpio" + m_p3_io + "/value";    
    // system(cmd.c_str());
}

bool PowerManager::f_cb_srv_set_p1(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) 
{

    std::string cmd;
    if(req.data)
    {
        cmd = "echo 1 > /sys/class/gpio/gpio" + m_p1_io + "/value"; 
        system(cmd.c_str());
        res.success = 1;
        printf("Port 1 power enabled\r\n");
        return true;
    }
    else
    {
        cmd = "echo 0 > /sys/class/gpio/gpio" + m_p1_io + "/value"; 
        system(cmd.c_str());
        res.success = 1;
        printf("Port 1 power disabled\r\n");
        return true;
    }
    return true;
}


bool PowerManager::f_cb_srv_get_state(
                        std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
{
    // char v[3];
    // std::string msg;
    // std::string cmd;
    // cmd = "cat /sys/class/gpio/gpio" + m_p1_io + "/value";
    // v[0] = system(cmd.c_str());
    // cmd = "cat /sys/class/gpio/gpio" + m_p2_io + "/value";
    // v[1] = system(cmd.c_str());
    // cmd = "cat /sys/class/gpio/gpio" + m_p3_io + "/value";
    // v[2] = system(cmd.c_str());

    // msg = "Port 1 state =" + v[0] + "| Device Name = " + m_p1_device_name + "\r\n"
    //     + "Port 2 state =" + v[1] + "| Device Name = " + m_p2_device_name + "\r\n"
    //     + "Port 3 state =" + v[2] + "| Device Name = " + m_p3_device_name + "\r\n";

    // res.success = 1;
    // res.message = msg;
    // printf("Power Manage Info:\r\n");
    // printf("%s", msg.c_str());
}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "power_manager");

    PowerManager i;

    ros::spin();

    return 0;
}
