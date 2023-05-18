#ifndef ALPHA_DRIVER_PICO_DRIVER_H
#define ALPHA_DRIVER_PICO_DRIVER_H

// c++
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
// 3rd party
#include <serial/serial.h>
// customized
#include <parameters.h>
#include <default.h>

class PicoDriver {

public:
    PicoDriver();

    ~PicoDriver();

    PicoDriver(const SerialParam &param);

    void initialize();
  
    void SendLine(const std::string &str);

    void SetCallback(decltype(serial_callback_) c) { serial_callback_  = c;}

    auto GetCallback() -> decltype(serial_callback_) {return serial_callback_;}

private:
    std::shared_ptr<serial::Serial> serial_;

    std::function <void(std::string)> serial_callback_;

    void ReceiveLoop();

};

#endif // ALPHA_DRIVER_PICO_DRIVER_H