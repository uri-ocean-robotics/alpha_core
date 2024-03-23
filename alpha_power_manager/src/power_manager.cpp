/*
This node toggles the GPIO on the PI
GPIO 9, 12, 13
To control the Power source on the power distribution board.
*/


#include "power_manager.hpp"
#include <iostream> 
#include <string>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

PowerManager::PowerManager()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->getParam("device_name", m_device_name);
    m_pnh->getParam("gpio_name", m_gpio_name);
    m_gpio_count = m_gpio_name.size();

    for (int i = 0; i <m_gpio_count; i++)
    {
        gpio_t g;
        g.device_name = m_device_name[i];
        g.gpio_name =  std::to_string(m_gpio_name[i]);
        g.service_name = "power_manager/set_power_gpio" + g.gpio_name;

        g.m_set_gpio = m_nh->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
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
        "power_manager/get_power_port_status",
        std::bind(
            &PowerManager::f_cb_srv_get_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_set_all_p_state = m_nh->advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
    (
        "power_manager/set_all_power_port",
        std::bind(
            &PowerManager::f_cb_srv_set_power_all,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    f_initialize_gpio();

}

bool PowerManager::f_initialize_gpio()
{
    std::string m_dir;
    int fd;

    // for each gpio we do the following
    for (int i=0; i<m_gpio_count; i++)
    {

        //check if GPIO already exported
        m_dir = "/sys/class/gpio/gpio" + gpio_vector[i].gpio_name + "/value";
        fd = open(m_dir.c_str(), O_WRONLY);
        if (fd >0) 
        {
            printf("%s exist\r\n", m_dir.c_str());
            close(fd);
            //No need to export
        }
        else
        {
            close(fd);
            //export the GPIO
            m_dir = "/sys/class/gpio/export";
            fd = open(m_dir.c_str(), O_WRONLY);
            if (fd == -1) {
                printf("Unable to open /sys/class/gpio/export\r\n");
            return false;
            }else
            {
                if (write(fd, gpio_vector[i].gpio_name.c_str(), gpio_vector[i].gpio_name.size()) != gpio_vector[i].gpio_name.size()) 
                {
                    printf("Error writing to /sys/class/gpio/export, gpio %s\r\n", gpio_vector[i].gpio_name.c_str());
                    return false;
                }
                
            }
            close(fd);
        }
    }

    sleep(1);
    for (int i=0; i<m_gpio_count; i++)
    {
        //set direction
        m_dir = "/sys/class/gpio/gpio" + gpio_vector[i].gpio_name + "/direction";
        fd = open(m_dir.c_str(), O_WRONLY);

        if (write(fd, "out", 3) != 3) 
        {
            printf("Error writing to %s \r\n", gpio_vector[i].gpio_name.c_str());
            close(fd);
            return false;
        }
        close(fd);
        //set value
        f_set_gpio_value("0", gpio_vector[i].gpio_name);
    }

}

bool PowerManager::f_set_gpio_value(std::string value, std::string gpio_name)
{
    std::string m_dir;
    int fd;

    m_dir = "/sys/class/gpio/gpio"+ gpio_name + "/value";
    fd = open(m_dir.c_str(), O_WRONLY);

    if (fd == -1) 
    {
        printf("Unable to open %s \r\n", m_dir.c_str());
        close(fd);
        return false;
    }
    if (write(fd, value.c_str() , value.size()) != 1) {
        printf("Error settting value to %s \r\n", m_dir.c_str());
        close(fd);
        return false;
    }
    close(fd);

}


bool PowerManager::f_cb_srv_set_power_all(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data)
    {
        for (int i=0; i < m_gpio_count; i++)
        {
            f_set_gpio_value("1", gpio_vector[i].gpio_name);
        }
        res.success = 1;
        res.message = "All power ports are enabled";
        return true;
    }
    else
    {
        for (int i=0; i < m_gpio_count; i++)
        {
            f_set_gpio_value("0", gpio_vector[i].gpio_name);
        }
        res.success = 1;
        res.message = "All power ports are disabled";
        return true;
    }
    return true;

}

bool PowerManager::f_cb_srv_set_power(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, std::string gpio_name) 
{

    if(req.data)
    {
        f_set_gpio_value("1", gpio_name);
        res.success = 1;
        res.message = "Port:" + gpio_name + " power enabled";
        return true;
    }
    else
    {
        f_set_gpio_value("0", gpio_name);
        res.success = 1;
        res.message = "Port:" + gpio_name + " power disabled";
        return true;
    }
    return true;
}


bool PowerManager::f_cb_srv_get_state(
                        std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res)
{
    std::string m_dir;
    int fd;
    int value;
    char v;
    std::string msg = "#Power Manager: ";

    for (int i=0; i<m_gpio_count; i++)
    {
        m_dir = "/sys/class/gpio/gpio" + gpio_vector[i].gpio_name + "/value";
        fd = open(m_dir.c_str(), O_RDONLY);
        read(fd, &v, sizeof(value));
        close(fd);
        msg = msg + "GPIO-"+ gpio_vector[i].gpio_name + "=" + v + " | ";
    }
    res.success = 1;
    res.message = msg;
    return true;
}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "power_manager");

    PowerManager i;

    ros::spin();

    return 0;
}
