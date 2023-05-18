
#include "ros/ros.h"

#include "alpha_driver_uart/pico_ros.h"

int main(int argc, char** argv) {

    ros::init(argc, argv,"alpha_Pico");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    PicoRos node(nh, nh_private);

    ros::spin();

    return 0;
}