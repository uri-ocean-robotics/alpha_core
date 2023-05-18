#ifndef ALPHA_DRIVER_PARAMETERS_H
#define ALPHA_DRIVER_PARAMETERS_H

#include <string>

struct SerialParam {
    std::string port;
    int baud;
    int timeout;
    int parity;
    int databits;
    int stopbits;
};

#endif // ALPHA_DRIVER_PARAMETERS_H