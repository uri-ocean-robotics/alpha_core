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
      Emir Cem Gezer <emircem.gezer@gmail.com>
      Lin Zhao <linzhao@uri.edu>
    Year: 2022-2023

    Copyright (C) 2022-2023 Smart Ocean Systems Laboratory
*/

#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "alpha/parameters.h"
#include "alpha/pwm_controller.h"
#include "alpha/manager.h"

const uint LED_PIN = 25;

Manager *manager = new Manager();

void ManagerThread() {
    manager->ReceiveMsg();
}

int main() {
    // some global pico setup ?
    stdio_init_all();

    // Set up LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);

    // core1 thread:
    //  - receive new messages from Pi
    //  - post-processing of those messages
    multicore_launch_core1(ManagerThread);

    while(true) {
        tight_loop_contents();
    }

    return 0;
}