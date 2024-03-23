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

    m_pnh->param<std::vector<std::string>>("device_name", m_device_name, "");
    m_pnh->param<std::vector<std::string>>("gpio_name", m_gpio_name, "9");
    m_pnh->param<int>("gpio_count", m_gpio_count, 5);


    // m_serial0_sub = m_nh.subscribe<std_msgs::String>("serial/0/in", 100, std::bind(&AlphaDriverRos::f_serial_callback, this, std::placeholders::_1, 0));

    for (int i = 0; i <m_gpio_count; i++)
    {
        thruster_t g;
        g.device_name = m_device_name[i];
        g.gpio_name = m_gpio_name[i];
        g.service_name = "set_power_" + g.device_name;
        g.m_set_gpio = m_nh->adverseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
                (
                    g.service_name, 
                    std::bind(
                        &PowerManager::f_cb_srv_set_power,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        g.gpio_name
                    )
                );
        gpio_vector.push_back(g);
    }

    m_get_p_state = m_nh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>
    (
        "get_power_status",
        std::bind(
            &PowerManager::f_cb_srv_get_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

bool PowerManager::f_initialize_gpio()
{
    std::string m_dir;
    int fd;

    //export gpios
    m_dir = "/sys/class/gpio/export"
    fd = open(m_dir, O_WRONLY);
    if (fd == -1) {
        perror("Unable to open /sys/class/gpio/export");
        return false;
    }
    close(fd);

    for (int i=0; i<m_gpio_count; i++)
    {
        fd = open(m_dir, O_WRONLY);
        //enable gpio
        if (write(fd, gpio_vector.gpio_name[i], gpio_vector.gpio_name[i].size()) 
                != gpio_vector.gpio_name[i].size()) 
        {
            perror("Error writing to /sys/class/gpio/export, gpio %s", gpio_vector.gpio_name[i].c_str());
            return false
        }
        close(fd);

        //set direction
        m_dir = "/sys/class/gpio/gpio" + gpio_vector.gpio_name[i] + "/direction";
        fd = open(m_dir, O_WRONLY);

        if (fd == -1) 
        {
            perror("Unable to open %s", m_dir.c_str());
        return false;
        }

        if (write(fd, "out", 3) != 3) {
            perror("Error writing to %s", m_dir.c_str());
            return false;
        }
        close(fd);

        //set the value to 0
        f_set_gpio_value("0", gpio_vector.gpio_name[i]);
    }

}

bool PowerManager::f_set_gpio_value(std::string value, std::string gpio_name)
{
    std::string m_dir;
    int fd;

    m_dir = "/sys/class/gpio/gpio"+ gpio_name + "/value";
    fd = open(m_dir, O_WRONLY);

    if (fd == -1) {
        perror("Unable to open %s", m_dir.c_str());
        return false
    }
    if (write(fd, value , value.size()) != 1) {
        perror("Error settting value to %s", m_dir.c_str());
        return false;
    }
    close(fd);

}

bool PowerManager::f_cb_srv_set_power(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, std::string gpio_name) 
{

    std::string cmd;
    if(req.data)
    {
        f_set_gpio_value("0", gpio_name);
        res.success = 1;
        res.message = "Port 1 power enabled\r\n";
        return true;
    }
    else
    {
        f_set_gpio_value("1", gpio_name);
        res.success = 1;
        res.message = "Port 1 power disabled\r\n";
        return true;
    }
    return true;
}


bool PowerManager::f_cb_srv_get_state(
                        std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
{
    std::string cmd;
    int fd;
    int value;
    char v[10];
    std::string msg = "";
     for (int i=0; i<m_gpio_count; i++)
     {
       cmd = "cat /sys/class/gpio/gpio" + gpio_vector.gpio_name[i] + "/value";
        v[i] = system(cmd.c_str());
        msg = msg + gpio_vector.device_name[i] + " Power port is" + v[i] + "\r\n";
     }

    res.success = 1;
    res.message = msg;
    printf("Power Manage Info:\r\n");
    printf("%s", msg.c_str());
}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "power_manager");

    PowerManager i;

    ros::spin();

    return 0;
}
