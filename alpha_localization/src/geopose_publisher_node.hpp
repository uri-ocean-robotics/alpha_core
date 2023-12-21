/*

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

    Author: Mingxi ZHou
    Email: mzhou@uri.edu
    Year: 2023

    Copyright (C) 2023 Smart Ocean Systems Laboratory

    This program subscribe to the odometry topic and convert
    position into lat lon altitude using toLL service.
    The values are published to a topic geolocation in 
    geographic_msgs/GeoPoseStamped message type
*/

#pragma once

#include "ros/ros.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "robot_localization/ToLL.h"

class OdomToGeoposeNode{

private:
    ros::NodeHandlePtr m_nh;

    ros::NodeHandlePtr m_pnh;

    ros::Publisher m_geopose_publisher;

    ros::Subscriber m_odom_subscriber;

    std::string m_toll_service;

    std::string m_odom_source;

    void f_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);


public:

    OdomToGeoposeNode();
    
};
