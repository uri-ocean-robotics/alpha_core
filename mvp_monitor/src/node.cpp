#include <ros/ros.h>

#include "monitor_abort.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "MVP_Monitor_Node"); 

  ROS_INFO("\n============================================\n"
           "   !!!!!!!!!! MVP Monitor Node Started !!!!!!!!!! "
           "\n============================================\n");
  
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  MonitorAbort abort_node(nh, nh_private);

  ros::spin();

  return 0;
}