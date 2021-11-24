#ifndef FINDTHEPATH_CLOCK_HPP
#define FINDTHEPATH_CLOCK_HPP

#include <chrono>
#include <iostream>

class Clock {
public:
    Clock() = default;
    void print() {
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
        start = end;
    }

private:
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> start = std::chrono::steady_clock::now();
};
#endif// FINDTHEPATH_CLOCK_HPP
