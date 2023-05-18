#include <pico_ros.h>
#include <unistd.h>

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
          std::bing(&PicoRos::CallbackPicoDriver, this, std::placeholders::_1));

    std::thread t(std::bind(&PicoRos::TestLoop, this));
    t.detach();
}

void PicoRos::LoadConfigure() {
    // serial confugure
    nh_private_.param<std::string>("Serial/port", serial_param_.port, DEFAULT_PORT);
    nh_private_.param<int>("Serial/baud", serial_param_.baud, DEFAULT_BAUD);
    nh_private_.param<std::string>("Serial/parity", serial_param_.parity, DEFAULT_PARITY);
    nh_private_.param<int>("Serial/databits", serial_param_.databits, DEFAULT_DATABITS);
    nh_private_.param<int>("Serial/stopbits", serial_param_.stopbits, DEFAULT_STOPBITS);
    nh_private_.param<int>("Serial/timeout", serial_param_.timeout, DEFAULT_TIMEOUT);
}

void PicoRos::CallbackRawNMEA(const std_msgs::String::ConstPtr &msg) {

}

void PicoRos::CallbackPicoDriver(const std::string &str) {
    printf("%s",str.c_str());
}

void PicoRos::TestLoop() {
    ros::Rate r(1);
    while(ros::ok()) {
        std::string str;

        str = "$1,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        str = "$2,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        str = "$3,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        str = "$4,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        str = "$5,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        str = "$6,abcdefghijklmnopqrstuvwxyz";
        pico_driver_->SendLine(str);
        usleep(1000);

        r.sleep();
    }  
}
