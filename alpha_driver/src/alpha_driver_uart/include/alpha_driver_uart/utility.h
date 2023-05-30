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

#ifndef UTILITY_TIMECOST_H
#define UTILITY_TIMECOST_H

#include <chrono>

class TimeCost{
public:
    TimeCost() {
        start = std::chrono::high_resolution_clock::now();  
    }

    // return time in second, the max time resoluation is microseconds
    double timecost() {
        end = std::chrono::high_resolution_clock::now();  

        double cost = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() * 1e-6;

        start = end;

        return cost;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif // UTILITY_TIMECOST_H
