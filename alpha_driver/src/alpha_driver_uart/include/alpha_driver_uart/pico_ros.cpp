#include <unistd.h>
#include <alpha_driver_uart/pico_ros.h>

PicoRos::PicoRos(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load configure
    LoadConfigure();

    // ros setup
    raw_nmea_sub = nh_.subscribe("driver/raw_nmea", 100, &PicoRos::CallbackRawNMEA, this);

    raw_nmea_pub = nh_.advertise<std_msgs::String>("driver/incoming_raw_nmea", 1000);

    // init
    pico_driver_ = std::make_shared<PicoDriver>(serial_param_);

    pico_driver_->SetCallback(
          std::bind(&PicoRos::CallbackPicoDriver, this, std::placeholders::_1));

    thruster_manager_ = std::make_shared<ThrusterManager>(nh_, nh_private_, pico_driver_);

    // std::thread t(std::bind(&PicoRos::TestLoop, this));
    // t.detach();
}

void PicoRos::LoadConfigure() {
    // serial configure
    nh_private_.param<std::string>("Serial/port", serial_param_.port, DEFAULT_PORT);
    nh_private_.param<int>("Serial/baud", serial_param_.baud, DEFAULT_BAUD);
    nh_private_.param<int>("Serial/timeout", serial_param_.timeout, DEFAULT_TIMEOUT);
    nh_private_.param<int>("Serial/parity", serial_param_.parity, DEFAULT_PARITY);
    nh_private_.param<int>("Serial/databits", serial_param_.databits, DEFAULT_DATABITS);
    nh_private_.param<int>("Serial/stopbits", serial_param_.stopbits, DEFAULT_STOPBITS);
}

void PicoRos::CallbackRawNMEA(const std_msgs::String::ConstPtr &msg) {

}

void PicoRos::CallbackPicoDriver(const std::string &str) {
    //! DEBUG:
    // printf("%s", str.c_str());

    // publish the raw string from pico
    std_msgs::String raw_msg;
    raw_msg.data = str;
    raw_nmea_pub.publish(raw_msg);
}

// void PicoRos::TestLoop() {
//     std::chrono::milliseconds dura(1);

//     ros::Rate r(1);
//     while(ros::ok()) {

//         std::string str;

//         str = "$ROS,1,abcdefghijk";
//         pico_driver_->SendLine(str);
//         std::this_thread::sleep_for(dura);

//         r.sleep();
//     }  
// }
