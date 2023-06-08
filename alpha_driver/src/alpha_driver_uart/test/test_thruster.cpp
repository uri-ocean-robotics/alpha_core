
// ros
#include <ros/ros.h>
// ros msg type header
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

class Testthruster
{
public:
    Testthruster(const ros::NodeHandle &nh,
                 const ros::NodeHandle &nh_private) 
            :nh_(nh), nh_private_(nh_private)
    {
        sub = nh_.subscribe<sensor_msgs::Joy> ("/joy", 10, &Testthruster::CallbackJoy, this);
        pub0 = nh_.advertise<std_msgs::Float64>("control/pwm_chan0", 10);
        pub1 = nh_.advertise<std_msgs::Float64>("control/pwm_chan1", 10);
        pub2 = nh_.advertise<std_msgs::Float64>("control/pwm_chan2", 10);
        pub3 = nh_.advertise<std_msgs::Float64>("control/pwm_chan3", 10);

        nh_private_.param<double>("factor", factor, 0.5);
    }

    ~Testthruster() {}

    void CallbackJoy(const sensor_msgs::Joy::ConstPtr& input);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber sub;

    ros::Publisher pub0;
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Publisher pub3;

    double factor;
};

void Testthruster::CallbackJoy(const sensor_msgs::Joy::ConstPtr& input) {
    
    auto ch0 = input->axes[0];
    auto ch1 = input->axes[1];
    auto ch2 = input->axes[2];
    auto ch3 = input->axes[3];

    std::cout<<std::endl;
    std::cout<<"ch0: " << ch0 << std::endl;
    std::cout<<"ch1: " << ch1 << std::endl;
    std::cout<<"ch2: " << ch2 << std::endl;
    std::cout<<"ch3: " << ch3 << std::endl;

    // send cmd to pico ros driver
    std_msgs::Float64 pwm_ch0;
    std_msgs::Float64 pwm_ch1;
    std_msgs::Float64 pwm_ch2;
    std_msgs::Float64 pwm_ch3;

    pwm_ch0.data = ch0*0.5;
    pwm_ch1.data = ch1*0.5;
    pwm_ch2.data = ch2*0.5;
    pwm_ch3.data = ch3*0.5;

    pub0.publish(pwm_ch0);
    pub1.publish(pwm_ch1);
    pub2.publish(pwm_ch2);
    pub3.publish(pwm_ch3);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Simple_Example_Node");

    ros::NodeHandle nh("");

    ros::NodeHandle nh_private("~");

    Testthruster  example(nh, nh_private);

    ros::spin();
    
    return 0;
}
