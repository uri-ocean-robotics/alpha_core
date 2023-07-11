
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
        pub0 = nh_.advertise<std_msgs::Float64>("joy_axes0", 10);
        pub1 = nh_.advertise<std_msgs::Float64>("joy_axes1", 10);
        pub2 = nh_.advertise<std_msgs::Float64>("joy_axes2", 10);
        pub3 = nh_.advertise<std_msgs::Float64>("joy_axes3", 10);

        nh_private_.param<double>("factor_joy0", factor_joy0, 0.5);
        nh_private_.param<double>("factor_joy1", factor_joy1, 0.5);
        nh_private_.param<double>("factor_joy2", factor_joy2, 0.5);
        nh_private_.param<double>("factor_joy3", factor_joy3, 0.5);
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

    double factor_joy0;
    double factor_joy1;
    double factor_joy2;
    double factor_joy3;
};

void Testthruster::CallbackJoy(const sensor_msgs::Joy::ConstPtr& input) {
    
    auto axes0 = input->axes[0];
    auto axes1 = input->axes[1];
    auto axes2 = input->axes[2];
    auto axes3 = input->axes[3];

    auto button_LB = input->buttons[4];
    // std::cout<<std::endl;
    // std::cout<<"axes0: " << axes0 << std::endl;
    // std::cout<<"axes1: " << axes1 << std::endl;
    // std::cout<<"axes2: " << axes2 << std::endl;
    // std::cout<<"axes3: " << axes3 << std::endl;

    // send cmd to pico ros driver
    std_msgs::Float64 pwm_axes0;
    std_msgs::Float64 pwm_axes1;
    std_msgs::Float64 pwm_axes2;
    std_msgs::Float64 pwm_axes3;

    pwm_axes0.data = axes0*factor_joy0;
    pwm_axes1.data = axes1*factor_joy1;
    pwm_axes2.data = axes2*factor_joy2;
    pwm_axes3.data = axes3*factor_joy3;

    if(button_LB == 1) {
        if(axes0 != 0.0) {
            pub0.publish(pwm_axes0);
        }

        if(axes1 != 0.0) {
            pub1.publish(pwm_axes1);
        }

        if(axes2 != 0.0) {
            pub2.publish(pwm_axes2);
        }        

        if(axes3 != 0.0) {
            pub3.publish(pwm_axes3);
        }
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Simple_Example_Node");

    ros::NodeHandle nh("");

    ros::NodeHandle nh_private("~");

    Testthruster  example(nh, nh_private);

    ros::spin();
    
    return 0;
}
