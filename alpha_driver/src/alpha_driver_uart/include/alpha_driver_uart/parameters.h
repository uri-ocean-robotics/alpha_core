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

struct ThrusterParam {
  int channels;
  int safety_timeout;
  int safety_rate;
};

#endif // ALPHA_DRIVER_PARAMETERS_H