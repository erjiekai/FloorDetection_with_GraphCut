#ifndef _TIMER_HPP_
#define _TIMER_HPP_

#include <chrono>

class Timer
{
private:
    std::chrono::high_resolution_clock::time_point starttime;
    std::chrono::high_resolution_clock::time_point stoptime;

public:
    Timer(){};
    ~Timer(){};

    void start()
    {
        starttime = std::chrono::high_resolution_clock::now();
    }

    double getInterval()
    {
        stoptime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::microseconds>(stoptime - starttime);

        return duration.count();
    }
};

#endif // _TIMER_HPP_