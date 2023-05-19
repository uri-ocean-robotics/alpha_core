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