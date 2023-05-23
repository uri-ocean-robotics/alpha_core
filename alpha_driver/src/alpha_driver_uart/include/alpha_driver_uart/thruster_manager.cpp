#include <alpha_driver_uart/thruster_manager.h>

ThrusterManager::ThrusterManager(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private,
                                 std::shared_ptr<PicoDriver> pico_driver) 
  :  nh_(nh), nh_private_(nh_private), pico_driver_(std::move(pico_driver))
{
    // load configure
    LoadConfigure();

    // initialize
    Initialize();

    // ros setup
    SetupROS();
}

void ThrusterManager::LoadConfigure() {
    // load configure
    nh_private_.param<int>("Thrusters/channels", thruster_param_.channels, DEFAULT_CHANNELS);
    nh_private_.param<int>("Thrusters/safety_timeout", thruster_param_.safety_timeout, DEFAULT_SAFETY_TIMEOUT);
    nh_private_.param<int>("Thrusters/safety_rate", thruster_param_.safety_rate, DEFAULT_SAFETY_RATE);

    //! DEBUG:
    // std::cout << "Thrusters" << std::endl;
    // std::cout << "  channels: " << thruster_param_.channels;
    // std::cout << "  safety_timeout: " << thruster_param_.safety_timeout;
    // std::cout << "  safety_rate: " << thruster_param_.safety_rate;
    // std::cout << std::endl;

    for(size_t i=0; i<thruster_param_.channels; i++) {
        pwm_control_t pwm;
        pwm.channel = i;
        pwm.topic = "control/pwm_chan" + std::to_string(i);
        pwm.mode = static_cast<uint8_t>(PwmMode::Thruster);
        pwm_control_.push_back(pwm);
    }
}

void ThrusterManager::Initialize() {
    //! NOTE: the pwm init moved to to pico
    // std::chrono::milliseconds dura_small(100);
    // for(const auto& i : pwm_control_) {
    //     InitializePWM(i.channel, i.mode);
    //     std::this_thread::sleep_for(dura_small);
    // }

    //! NOTE: each pico pwm device has it's own saftey check timer, do we really need this one ?????
    // start a safety loop to monitor latest thruster pwm commands
    std::thread t(std::bind(&ThrusterManager::SafetyLoop, this));
    t.detach();
}

void ThrusterManager::SetupROS() {
    // pub
    status_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("driver/thrust_status", 1000);

    // sub for multiple thresters
    for(const auto& i : pwm_control_) {
        auto sub = nh_.subscribe<std_msgs::Float64>(
            i.topic,
            6,
            std::bind(
                &ThrusterManager::CallbackPWM,
                this,
                std::placeholders::_1,
                i.channel,
                i.mode
            )
        );

        pwm_subs_.emplace_back(sub);
    }

}

void ThrusterManager::CallbackPWM(const std_msgs::Float64::ConstPtr &msg, 
                                  uint16_t channel, 
                                  uint8_t mode) {
    last_pwm_time_ = ros::Time::now();

    SendPWM(channel, msg->data);
}

void ThrusterManager::SafetyLoop() {

    int sleep_time = 1.0 / thruster_param_.safety_rate * 1000;
    std::chrono::milliseconds dura_small(1);
    std::chrono::milliseconds dura_large(sleep_time);

    while(true) {

        auto dt = ros::Time::now() - last_pwm_time_;

        if(dt.toSec() > thruster_param_.safety_timeout) {
            for(const auto& i : pwm_control_) {
                // send stop pwm: usually take 100 microseconds 
                SendPWM(i.channel, 0);

                // sleep for a very short amount of time, really need this ?
                std::this_thread::sleep_for(dura_small);
            }
        }

        std::this_thread::sleep_for(dura_large);
    }    
}

void ThrusterManager::SendPWM(int channel, double pwm) {
    // construct NMEA string
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_CMD, NMEA_PWM_CMD, channel, pwm);

    // serial send 
    auto size = pico_driver_->SendLine(std::string(msg.get_raw()));
}

void ThrusterManager::InitializePWM(int channel, int mode) {
    // construct NMEA string
    NMEA msg;
    msg.construct(NMEA_FORMAT_PWM_INIT, NMEA_PWM_INITIALIZE, channel, mode);

    // serial send
    auto size = pico_driver_->SendLine(std::string(msg.get_raw()));
}