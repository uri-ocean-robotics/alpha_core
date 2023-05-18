#ifndef ALPHA_DRIVER_PARAMETERS_H
#define ALPHA_DRIVER_PARAMETERS_H

#include <string>

struct SerialParam {
    std::string port;
    uint16_t baud;
    std::string parity;
    uint8_t databits;
    uint8_t stopbits;
    uint16_t timeout;
};

#endif // ALPHA_DRIVER_PARAMETERS_H