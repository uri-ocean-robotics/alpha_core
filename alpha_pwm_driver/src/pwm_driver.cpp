
#include "pwm_driver.h"
#include <iostream> 
#include <string>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

PWMDriver::PWMDriver()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));
  
    m_pnh->param<double>("max_ms_offset", m_max_ms_offset, 0.4);
    m_pnh->param<double>("ms_center", m_ms_center, 1.5);
    m_pnh->param<std::string>("topic_prefix", m_prefix, "");

    m_pnh->getParam("topic_list", m_topic_list);
    m_pnh->getParam("channel_list", m_channel_list);
    

    //initialize PCA9685
    pca.set_pwm_freq(50.0);

    for (int i = 0; i <m_channel_list.size(); i++)
    {        
        std::string t_name = m_prefix + m_topic_list[i];
        m_sub_list[i] =  m_nh->subscribe<std_msgs::Float64>(t_name, 10, 
                    std::bind(&PWMDriver::f_cb_pwm_ch, 
                                this, 
                                std::placeholders::_1, 
                                m_channel_list[i])
                                );
        //set all channel to 1500 us
        pca.set_pwm_ms(m_channel_list[i], 1.5);
    }

}

void PWMDriver::f_cb_pwm_ch(const std_msgs::Float64::ConstPtr& msg, int ch)
{
    //data is from -1 to 1 mapped into with m_max_offset from 1.5
    pca.set_pwm_ms(ch, m_ms_center + msg->data * m_max_ms_offset);
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "pwm_driver");

    PWMDriver i;

    ros::spin();

    return 0;
}
