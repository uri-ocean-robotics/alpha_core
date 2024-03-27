#ifndef PWM_DRIVER_
#define PWM_DRIVER_

#include <memory>
#include <string>
#include "ros/ros.h"
#include "PCA9685.h"
#include "std_msgs/Float64.h"

class PCA9685;
class PWMDriver
{
    private:
        ros::NodeHandlePtr m_nh;
        ros::NodeHandlePtr m_pnh;

        std::vector<std::string> m_topic_list;
        std::vector<int> m_channel_list;
        std::vector<ros::Subscriber> m_sub_list;

        double m_max_ms_offset = 0.4;  //default 0.4 ms
        double m_ms_center = 1.5;   //1.5 ms
        void f_cb_pwm_ch(const std_msgs::Float64::ConstPtr& msg, int ch);

        PCA9685 pca{};

    public:
        PWMDriver();
};


#endif // PWM_DRIVER