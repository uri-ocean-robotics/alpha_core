/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2023-2023

    Copyright (C) 2023-2023 Smart Ocean Systems Laboratory
*/

#ifndef ALPHA_DRIVER_PICO_DRIVER_H
#define ALPHA_DRIVER_PICO_DRIVER_H

// c++
#include <thread>
#include <chrono>
#include <memory>
#include <functional>
#include <iostream>
#include <atomic>
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

    std::atomic<bool> close_;
    
public:
    PicoDriver();

    ~PicoDriver();

    PicoDriver(const SerialParam &param);

    void initialize();
  
    size_t SendLine(const std::string &str);

    void SetCallback(decltype(serial_callback_) c) { serial_callback_  = c;}

    auto GetCallback() -> decltype(serial_callback_) {return serial_callback_;}

};

#endif // ALPHA_DRIVER_PICO_DRIVER_H