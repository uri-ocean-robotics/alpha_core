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

    Author: Mingxi Zhou
    Email: mzhou
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

#include "world_odom_transform.hpp"
#include <tf2_ros/transform_broadcaster.h>

WorldOdomTransform::WorldOdomTransform(){
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));
    
    m_pnh->param<std::string>("world", m_world_frame, "world");

    m_pnh->param<std::string>("odom", m_odom_frame, "odom");

    m_pnh->param<std::string>("tf_prefix", m_tf_prefix, "");

    m_pnh->param<double>("mag_declination", m_mag_declination, 0.0);
    //mag_north - true north in ENU frame.

    m_pnh->param<double>("acceptable_var", m_acceptable_var, 0.0);

    m_pnh->param<double>("position_accuracy", m_position_accuracy, 0.0);

    m_pnh->param<double>("max_gps_wait_time", m_gps_wait_time, 60.0);

    m_pnh->param<double>("datum_latitude", m_datum_latitude, 0.0);

    m_pnh->param<double>("datum_longitude", m_datum_longitude, 0.0);

    m_pnh->param<double>("datum_altitude", m_datum_altitude, 0.0);

    m_pnh->param<bool>("publish_tf", m_publish_tf, true);
    
    m_datum.latitude = m_datum_latitude;
    m_datum.longitude = m_datum_longitude;
    m_datum.altitude = m_datum_altitude;

    if(m_datum.latitude==0)
    {
        ROS_WARN("Datum is not set please set the parameters");
        m_datum_set = false;
    }
    m_tf_set = false;
    m_world_frame = m_tf_prefix + "/" + m_world_frame;

    m_odom_frame = m_tf_prefix + "/" + m_odom_frame;

    m_gps_odom_publisher = m_nh->advertise<nav_msgs::Odometry>("gps/odometry", 10);
    
    m_datum_publisher = m_nh->advertise<geographic_msgs::GeoPoint>("gps/datum",10);


    m_gps_fix_subscriber = m_nh->subscribe("gps/fix", 10, 
                                &WorldOdomTransform::f_cb_gps_fix, this);
    m_odom_subscriber = m_nh->subscribe("odometry", 10, 
                                &WorldOdomTransform::f_cb_odom, this);

    /**
     * Initialize services
     */
    fromLL_server = m_pnh->advertiseService<robot_localization::FromLL::Request,
        robot_localization::FromLL::Response>
        (
        "fromLL",
        std::bind(&WorldOdomTransform::f_cb_fromLL_srv,
            this,std::placeholders::_1,std::placeholders::_2
        )
        );

    toLL_server = m_pnh->advertiseService<robot_localization::ToLL::Request,
        robot_localization::ToLL::Response>
        (
        "toLL",
        std::bind(&WorldOdomTransform::f_cb_toLL_srv,
            this,
            std::placeholders::_1,std::placeholders::_2
        )
        );

    reset_tf_server = m_pnh->advertiseService<std_srvs::Trigger::Request,
        std_srvs::Trigger::Response>
        (
        "reset_datum",
        std::bind(&WorldOdomTransform::f_cb_reset_datum_srv,
            this,std::placeholders::_1,std::placeholders::_2
        )
        );

    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );
    
}

void WorldOdomTransform::f_cb_gps_fix(const sensor_msgs::NavSatFix& msg)
{
    //compute the latitude longitude in the world frame using datum.
    geometry_msgs::Point map_point;
    nav_msgs::Odometry gps_world;
    geographic_msgs::GeoPoint ll_point;

    m_gps = msg;
    m_odom_gps = m_odom; //map the most recent odom;


    if(m_datum_set)
    {
        m_datum_publisher.publish(m_datum); 
        return;
    }
    // printf("Got GPS\r\n");
    //only do the following if the tf between world and odom are set.
    if(m_tf_set)
    {
        ll_point.latitude = msg.latitude;
        ll_point.longitude = msg.longitude;
        ll_point.altitude = msg.altitude;
        //Get x and y from lattiude and longitude. x->east, y->north
        f_ll2dis(ll_point, map_point);
        //convert distance from gps into odom frame using mag_declination.
        try {        
            auto tf_w2o = m_transform_buffer.lookupTransform(
                m_odom_frame,
                m_world_frame,
                ros::Time(0)
            );
            auto tf_eigen = tf2::transformToEigen(tf_w2o);

            Eigen::Vector3d p_odom;
            p_odom = tf_eigen.rotation() * Eigen::Vector3d(map_point.x, map_point.y, map_point.z) + tf_eigen.translation();
            // printf("Latlon=%lf, %lf\r\n", ll_point.latitude, ll_point.longitude);
            // printf("T_matrix=%lf,%lf\r\n", tf_eigen.translation().x(), tf_eigen.translation().y());
            // printf("map_point=%lf,%lf\r\n", map_point.x, map_point.y);
            // printf("translated=%lf, %lf\r\n", p_odom.x(), p_odom.y());
            // printf("odom_point=%lf,%lf\r\n",m_odom.pose.pose.position.x, m_odom.pose.pose.position.y);
            gps_world.pose.pose.position.x = p_odom.x();
            gps_world.pose.pose.position.y = p_odom.y();
            gps_world.header.frame_id = m_odom_frame;
            gps_world.header.stamp = msg.header.stamp;
            gps_world.pose.covariance[0] = pow(m_position_accuracy,2);
            gps_world.pose.covariance[1] = 0;
            gps_world.pose.covariance[2] = 0;
            gps_world.pose.covariance[6] =0;
            gps_world.pose.covariance[7] =  pow(m_position_accuracy,2);
            gps_world.pose.covariance[8] = 0;
            gps_world.pose.covariance[12] = 0;
            gps_world.pose.covariance[13] = 0;
            gps_world.pose.covariance[14] =  pow(m_position_accuracy,2);

            m_gps_odom_publisher.publish(gps_world);       
        } catch(tf2::TransformException &e) {
            ROS_WARN_STREAM_THROTTLE(10, std::string("Can't get the tf from world to odom") + e.what());
        }
    }
    else
    {
        if(m_gps.position_covariance[0]<m_acceptable_var && m_gps.position_covariance[4]<m_acceptable_var)
        {
            f_set_tf();
        }
        else{
            ROS_INFO("GPS fix covariance is more than %lf\r\n", m_acceptable_var);
            return;
        }
    }
}

bool WorldOdomTransform::f_set_tf()
{
    //check the quality of the gps if the x and y variance is good enough?

    geographic_msgs::GeoPoint ll_point;
    geometry_msgs::Point map_point;

    ll_point.latitude = m_gps.latitude;
    ll_point.longitude = m_gps.longitude;
    ll_point.altitude = m_gps.altitude;
    //Get x and y from lattiude and longitude. x->east, y->north
    f_ll2dis(ll_point, map_point);

    // printf("tf update\r\n");
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = m_world_frame;
    transformStamped.child_frame_id = m_odom_frame;
    double dx = -m_odom_gps.pose.pose.position.x + map_point.x;
    double dy = -m_odom_gps.pose.pose.position.y + map_point.y;
    // printf("dx=%lf, dy =%lf\r\n", dx, dy);
    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;
    transformStamped.transform.translation.z = m_odom_gps.pose.pose.position.z - 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_mag_declination);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);

    ROS_INFO("TF Between world and odom is set\r\n");
    m_tf_set = true;
    
}


void WorldOdomTransform::f_cb_odom(const nav_msgs::Odometry& msg)
{
    // printf("got odometry\r\n");
    
    m_odom = msg;
    //if tf is set i will keep setting the tf
    if(m_tf_set)
    {
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
    }
}

bool WorldOdomTransform::f_cb_reset_datum_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{

}

bool WorldOdomTransform::f_cb_fromLL_srv(robot_localization::FromLL::Request &req, robot_localization::FromLL::Response &resp)
{
    f_ll2dis(req.ll_point, resp.map_point);
    return true;

}

bool WorldOdomTransform::f_cb_toLL_srv(robot_localization::ToLL::Request &req, robot_localization::ToLL::Response &resp)
{
    f_dis2ll(req.map_point, resp.ll_point);
    return true;
}

void WorldOdomTransform::f_ll2dis(geographic_msgs::GeoPoint ll_point, geometry_msgs::Point& map_point)
{
    double north = m_earthR*(ll_point.latitude - m_datum.latitude)/180.0*M_PI;
    double east = m_earthR*cos(m_datum.latitude/180.0*M_PI) * (ll_point.longitude - m_datum.longitude)/180.0*M_PI;
    map_point.x = east;
    map_point.y = north;
    map_point.z = ll_point.altitude;
    //in world frame.

}

void WorldOdomTransform::f_dis2ll(geometry_msgs::Point map_point, geographic_msgs::GeoPoint& ll_point)
{
    //from world frame
    double lat = m_datum.latitude + map_point.y/m_earthR * 180.0/M_PI;
    double lon = m_datum.longitude + map_point.x/(m_earthR*cos(m_datum.latitude/180.0*M_PI)) * 180.0/M_PI;
    ll_point.latitude = lat;
    ll_point.longitude = lon;
    ll_point.altitude = map_point.z;
}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "gps_transform");

    WorldOdomTransform d;
    
    ros::spin();

    return 0;
}