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
