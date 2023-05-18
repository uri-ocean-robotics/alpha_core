#ifndef ALPHA_DRIVER_THRUSTER_MANAGER_H
#define ALPHA_DRIVER_THRUSTER_MANAGER_H

// c++
#include <vector>
// ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
// customized
#include <alpha_driver_uart/pico_driver.h>
#include <alpha_driver_uart/utility.h>
#include <alpha/common/dictionary.h>
#include <nmea/nmea.h>

class ThrusterManager {
  
private:
    ros::NodeHandle nh_;

    ros::NodeHandle nh_private_;

    ros::Publisher status_pub_;

    std::vector<ros::Subscriber> pwm_subs_;

    std::shared_ptr<PicoDriver> pico_driver_;

    ThrusterParam thruster_param_;

    typedef struct pwm_control_t : pwm_t {
        std::string topic;
    } pwm_control_t;

    std::vector<pwm_control_t> pwm_control_;

    ros::Time last_pwm_time_;

    /** setup functions **/

    void LoadConfigure();

    void SetupROS();
    
    void Initialize();

    /** communication functions **/

    void InitializePWM(int channel, int mode);

    void SafetyLoop();

    void CallbackPWM(const std_msgs::Float64::ConstPtr &msg, uint16_t channel, uint8_t mode);

    void SendPWM(int channel, double pwm);

public:
    ThrusterManager(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private,
                    std::shared_ptr<PicoDriver> pico_driver);

    ~ThrusterManager(){}

};

#endif // ALPHA_DRIVER_THRUSTER_MANAGER_H