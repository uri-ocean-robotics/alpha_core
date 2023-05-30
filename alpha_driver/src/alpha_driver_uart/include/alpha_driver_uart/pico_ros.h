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

#ifndef ALPHA_DRIVER_PICO_ROS_H
#define ALPHA_DRIVER_PICO_ROS_H

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
// customized
#include <alpha_driver_uart/pico_driver.h>
#include <alpha_driver_uart/thruster_manager.h>

class PicoRos {
private:
    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Subscriber raw_nmea_sub;

    ros::Publisher raw_nmea_pub;

    std::shared_ptr<PicoDriver> pico_driver_;

    std::shared_ptr<ThrusterManager> thruster_manager_;

    SerialParam serial_param_;

    void LoadConfigure();

    void CallbackRawNMEA(const std_msgs::String::ConstPtr &msg);

    void CallbackPicoDriver(const std::string &str);

    // void TestLoop();
        
public:
    PicoRos(const ros::NodeHandle &nh,
            const ros::NodeHandle &nh_private);

    ~PicoRos(){}

};

#endif // ALPHA_DRIVER_PICO_ROS_H