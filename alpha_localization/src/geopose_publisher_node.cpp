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

#include "geopose_publisher_node.hpp"

OdomToGeoposeNode::OdomToGeoposeNode()
{
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->param<std::string>("toll_service", m_toll_service, "toLL");
    m_pnh->param<std::string>("odometry_source", m_odom_source, "odometry/filtered/local");


    m_odom_subscriber = m_nh->subscribe(
        m_odom_source, 10, &OdomToGeoposeNode::f_odom_callback, this);

    m_geopose_publisher = m_nh->advertise
        <geographic_msgs::GeoPoseStamped>("odometry/geopose",10);
}

void OdomToGeoposeNode::f_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geographic_msgs::GeoPoseStamped geopose;
    //check to_LL service
    if(!ros::service::exists(m_toll_service, false)) {
        // std::cout << "The To_LL service is not available from " << m_toll_service << std::endl;
    }
    else{
        //call the service 
        robot_localization::ToLL ser;
        ser.request.map_point.x = msg->pose.pose.position.x;
        ser.request.map_point.y = msg->pose.pose.position.y;
        ser.request.map_point.z = msg->pose.pose.position.z;

        if(!ros::service::call(m_toll_service, ser)) {
            std::cout << "Failed to compute the latitude and longitude from odom" << std::endl;
        }
        else{
        //map response into our service response.
        geopose.pose.position.latitude = ser.response.ll_point.latitude;
        geopose.pose.position.longitude = ser.response.ll_point.longitude;
        geopose.pose.position.altitude = ser.response.ll_point.altitude;
        geopose.pose.orientation.x = msg->pose.pose.orientation.x;
        geopose.pose.orientation.y = msg->pose.pose.orientation.y;
        geopose.pose.orientation.z = msg->pose.pose.orientation.z;
        geopose.pose.orientation.w = msg->pose.pose.orientation.w;
        geopose.header= msg->header;
        m_geopose_publisher.publish(geopose);
        }
    }
   
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "odom_to_geopose");

    OdomToGeoposeNode odom2geopose;

    ros::spin();

    return 0;
}