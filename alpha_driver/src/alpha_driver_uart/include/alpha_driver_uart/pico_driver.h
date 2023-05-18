#ifndef ALPHA_DRIVER_PICO_DRIVER_H
#define ALPHA_DRIVER_PICO_DRIVER_H

// c++
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <iostream>
// 3rd party
#include <serial/serial.h>
// customized
#include <alpha_driver_uart/parameters.h>
#include <alpha_driver_uart/default.h>

class PicoDriver {

private:
    std::shared_ptr<serial::Serial> serial_;

    std::function <void(std::string)> serial_callback_;

    void ReceiveLoop();
    
public:
    PicoDriver();

    ~PicoDriver(){}

    PicoDriver(const SerialParam &param);

    void initialize();
  
    void SendLine(const std::string &str);

    void SetCallback(decltype(serial_callback_) c) { serial_callback_  = c;}

    auto GetCallback() -> decltype(serial_callback_) {return serial_callback_;}

};

#endif // ALPHA_DRIVER_PICO_DRIVER_H