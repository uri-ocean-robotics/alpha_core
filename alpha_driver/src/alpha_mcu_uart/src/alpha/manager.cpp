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

#include "manager.h"

Manager::Manager() {

    // Setup UART for normal communication and debug
    if(COMM_NORMAL_TYPE == 0) {
        // setup uart
        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(1, GPIO_FUNC_UART);
        uart_init(uart0, COMM_NORMAL_BAUD);
        uart_set_hw_flow(uart0, false, false);
        uart_set_fifo_enabled(uart0, false);
    }
    else if(COMM_NORMAL_TYPE == 1) {
        // setup uart
        gpio_set_function(8, GPIO_FUNC_UART);
        gpio_set_function(9, GPIO_FUNC_UART);
        uart_init(uart1, COMM_NORMAL_BAUD);
        uart_set_hw_flow(uart1, false, false);
        uart_set_fifo_enabled(uart1, false);        
    }

    if(COMM_DEBUG_TYPE == 0) {
        // setup uart
        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(1, GPIO_FUNC_UART);
        uart_init(uart0, COMM_DEBUG_BAUD);
        uart_set_hw_flow(uart0, false, false);
        uart_set_fifo_enabled(uart0, false);
    }
    else if(COMM_DEBUG_TYPE==1) {
        // setup uart
        gpio_set_function(8, GPIO_FUNC_UART);
        gpio_set_function(9, GPIO_FUNC_UART);
        uart_init(uart1, COMM_DEBUG_BAUD);
        uart_set_hw_flow(uart1, false, false);
        uart_set_fifo_enabled(uart1, false);
    }

    // setup PWM devices
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_0, 0));
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_1, 1));
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_2, 2));
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_3, 3));
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_4, 4));
    m_pwm_channels.push_back(new PwmController(PWM_CHANNEL_PIN_5, 5));
    for(const auto& ch : m_pwm_channels) {
        ch->initialize();
    }

    // setup reporter for pwm 
    // add_repeating_timer_ms(PWM_REPORT_PERIOD, ReportPWM, this, &m_reporter_timer);

}

Manager::~Manager() {

    for(auto ch : m_pwm_channels) {
        delete ch;
    }

    m_pwm_channels.clear();
}

// bool Manager::ReportPWM(struct repeating_timer *t) {
//     auto self = (Manager*)t->user_data;

//     for(const auto& ch : self->m_pwm_channels) {
//         // skip it if the pwm controller is not enabled
//         if(!ch->get_enable()) {
//             continue;
//         }

//         NMEA *msg = new NMEA();
//         msg->construct(NMEA_FORMAT_PWM_REPORT,
//                        NMEA_PWM_REPORT,
//                        ch->get_channel(),
//                        ch->get_current(),
//                        ch->get_mode(),
//                        ch->get_enable()
//         );        

//         std::string str = msg->get_raw();
//         self->SendMsgLine(str);

//         delete msg;        
//     }

//     return true;
// }

// only receive and parse the port for the normal commm
void Manager::ReceiveMsg() {
    // for the uart normal comm
    if(COMM_NORMAL_TYPE==0) {
        m_uart_id = uart0;
    }
    else {
        m_uart_id = uart1;
    }

    while(true) {
        
        // use usb for normal commm
        if(COMM_NORMAL_TYPE==2) {
            std::string str;
            std::cin >> str;
            ParseMsg(str);

            continue;
        }
        
        // use uart for normal comm
        while (uart_is_readable(m_uart_id)) {
            // get each char
            uint8_t c = uart_getc(m_uart_id);

            if(c == '\n') {
                m_str.push_back(c);
                m_str_queue.push(m_str);
                m_str.clear();
            }
            else {
                m_str.push_back(c);
            }

            // check if the string can be sent
            if(m_str_queue.size() > 0) {
                // parse the coming msg into specific cmd
                ParseMsg(m_str_queue.front());
                // delete the used msg
                m_str_queue.pop();
            } 
        }             
    }
}

void Manager::ParseMsg(const std::string &str) {
    //! TODO: send bad msg back to pi: $PCIO,str*00\r\n

    NMEA msg(str.c_str());

    msg.parse();

    if(!msg.get_valid()) {
        auto str_invaild = "X: " + str; 
        SendMsg(str_invaild);
        return;
    }

    if (strcmp(msg.get_cmd(), NMEA_PWM_CMD) == 0) {
        // bad
        if(msg.get_argc() != 2) {
            return;
        }

        // get info from parsed msg
        int channel;
        float signal;
        sscanf(msg.get_data(), "%*[^,],%d,%f", &channel, &signal);

        // do something
        // SetPWM(channel,signal);
        m_pwm_channels[channel]->set_pwm(signal);
    }
    // else if (strcmp(msg.get_cmd(), NMEA_PWM_INITIALIZE) == 0) {
    //     // bad
    //     if(msg.get_argc() != 2) {
    //         return;
    //     }

    //     // get info from parsed msg
    //     int channel;
    //     int mode;
    //     sscanf(msg.get_data(), "%*[^,],%d,%d", &channel, &mode);

    //     // do something
    //     SetPWMInitialized(channel, mode); 
    // }    
    else {
        // report 
        auto str = "Wrong";
        SendMsgLine(str);
    }
}

bool Manager::SetPWMInitialized(int channel, int mode) {
    // report
    auto str = "Ini: " + std::to_string(channel) + 
               " Mode: " + std::to_string(mode);
    SendMsgLine(str);

    m_pwm_channels[channel]->set_mode(mode);
    m_pwm_channels[channel]->enable();

    return true;
}

bool Manager::SendMsg(const std::string &str, bool debug) {
    bool flag = false;

    if(!debug) {
        // use the normal comm port
        if(COMM_NORMAL_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str.c_str());
                flag = true;
            }            
        }
        else if(COMM_NORMAL_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str.c_str());
                flag = true;
            }            
        }
        else {
            // use USB
            std::cout << str;
            flag = true;            
        }
    }
    else {
        // use the debug comm port
        if(COMM_DEBUG_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str.c_str());
                flag = true;
            }            
        }
        else if(COMM_DEBUG_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str.c_str());
                flag = true;
            }            
        }
        else {
            // use USB
            std::cout << str;
            flag = true;            
        }        
    }

    return flag;
}

bool Manager::SendMsgLine(const std::string &str, bool debug) {
    bool flag = false;

    std::string str_line = str + "\r\n";


    if(!debug) {
        // use the normal comm port
        if(COMM_NORMAL_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str_line.c_str());
                flag = true;
            }            
        }
        else if(COMM_NORMAL_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str_line.c_str());
                flag = true;
            }            
        }
        else {
            // use USB
            std::cout << str_line;
            flag = true;            
        }

    }
    else {
        // use the debug comm port
        if(COMM_DEBUG_TYPE==0){
            // use UART0
            if(uart_is_writable(uart0)) {
                uart_puts(uart0, str_line.c_str());
                flag = true;
            }            
        }
        else if(COMM_DEBUG_TYPE==1) {
            // use UART1
            if(uart_is_writable(uart1)) {
                uart_puts(uart1, str_line.c_str());
                flag = true;
            }            
        }
        else {
            // use USB
            std::cout << str_line;
            flag = true;            
        }        
    }    

    return flag;
}
