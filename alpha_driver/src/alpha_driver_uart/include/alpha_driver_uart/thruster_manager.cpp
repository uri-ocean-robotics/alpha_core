/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2023-2023

    Copyright (C) 2023-2023 Smart Ocean Systems Laboratory
*/

#include <alpha_driver_uart/thruster_manager.h>

ThrusterManager::ThrusterManager(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private,
                                 std::shared_ptr<PicoDriver> pico_driver) 
  :  nh_(nh), nh_private_(nh_private), pico_driver_(std::move(pico_driver))
{
    // load configure
    LoadConfigure();

    // initialize
    Initialize();

    // ros setup
    SetupROS();
}

void ThrusterManager::LoadConfigure() {
    // system config
    nh_private_.param<int>("Thrusters/safety_timeout", 
        system_param_.safety_timeout, DEFAULT_SAFETY_TIMEOUT);
    nh_private_.param<int>("Thrusters/safety_rate", 
        system_param_.safety_rate, DEFAULT_SAFETY_RATE);

    // pwm config
    std::vector<std::string> keys;

    nh_.getParamNames(keys);

    auto ns = nh_private_.getNamespace();

    auto param_name = ns + "/" + DEFAULT_CONF_PWM_CONTROL;    

    for(const auto &i : keys) {
        if(i.find(param_name) != std::string::npos) {
            auto pp = i.substr(param_name.size() + 1, i.size());
            auto name = pp.substr(0, pp.find('/'));
            auto param = pp.substr(pp.find('/') + 1, pp.size());

            if(param == DEFAULT_CONF_PWM_CHANNEL) {
                int channel;
                nh_.getParam(i, channel);
                pwm_control_[name].channel = channel;
            } else if (DEFAULT_CONF_PWM_TOPIC) {
                std::string topic;
                nh_.getParam(i, topic);
                pwm_control_[name].topic = topic;
            } else if (param == DEFAULT_CONF_PWM_MODE) {
                std::string mode;
                nh_.getParam(i, mode);
                if(mode == DEFAULT_CONF_PWM_MODE_OPT_THRUSTER) {
                    pwm_control_[name].mode = static_cast<uint8_t>(PwmMode::Thruster);
                } else if (mode == DEFAULT_CONF_PWM_MODE_OPT_PURE) {
                    pwm_control_[name].mode = static_cast<uint8_t>(PwmMode::Pure);
                } else {

                }
            } else {

            }
        }
    } 

    //! DEBUG:
    // for(const auto& i : pwm_control_) {
    //     printf("name=%s, chan=%d, mode=%d, topic=%s\n", 
    //             i.first.c_str(), i.second.channel,
    //             i.second.mode, i.second.topic.c_str());
    // }

    for(const auto &i : pwm_control_) {
        for(const auto &j : pwm_control_) {
            if(i.first == j.first) {
                continue;
            }

            if(i.second.topic.empty()) {
                throw alpha_driver_ros_exception("empty topic name");
            }

            if(i.second.channel == j.second.channel) {
                throw alpha_driver_ros_exception("multiple thrusters with same pwm channel found");
            }

            if(i.second.topic == j.second.topic) {
                throw alpha_driver_ros_exception("multiple thrusters with same topic id found");
            }
        }
    }    
}

void ThrusterManager::Initialize() {

    //! NOTE: each pico pwm device has it's own saftey check timer, 
    //        do we really need this one ?????
    // start a safety loop to monitor latest thruster pwm commands
    std::thread t(std::bind(&ThrusterManager::SafetyLoop, this));
    t.detach();
}

void ThrusterManager::SetupROS() {
    // pub
    status_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_status", 1000);

    // sub for multiple thresters
    for(const auto& i : pwm_control_) {
        auto sub = nh_.subscribe<std_msgs::Float64>(
            i.second.topic,
            6,
            std::bind(
                &ThrusterManager::CallbackPWM,
                this,
                std::placeholders::_1,
                i.second.channel,
                i.second.mode
            )
        );

        pwm_subs_.emplace_back(sub);
    }

}

void ThrusterManager::CallbackPWM(const std_msgs::Float64::ConstPtr &msg, 
                                  uint16_t channel, 
                                  uint8_t mode) {
    last_pwm_time_ = ros::Time::now();

    SendPWM(channel, msg->data);
}

void ThrusterManager::SafetyLoop() {

    int sleep_time = 1.0 / system_param_.safety_rate * 1000;
    std::chrono::milliseconds dura_small(1);
    std::chrono::milliseconds dura_large(sleep_time);

    while(true) {

        auto dt = ros::Time::now() - last_pwm_time_;

        if(dt.toSec() > system_param_.safety_timeout) {
            for(const auto& i : pwm_control_) {
                // send stop pwm: usually take 100 microseconds 
                // SendPWM(i.second.channel, 0);

                // sleep for a very short amount of time, really need this ?
                std::this_thread::sleep_for(dura_small);
            }
        }

        std::this_thread::sleep_for(dura_large);
    }    
}

void ThrusterManager::SendPWM(int channel, double pwm) {
    // construct NMEA string
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_CMD, NMEA_PWM_CMD, channel, pwm);

    // serial send 
    auto size = pico_driver_->SendLine(std::string(msg.get_raw()));
}

void ThrusterManager::InitializePWM(int channel, int mode) {
    // construct NMEA string
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_INIT, NMEA_PWM_INITIALIZE, channel, mode);

    // serial send
    auto size = pico_driver_->SendLine(std::string(msg.get_raw()));
}