#ifndef FINDTHEPATH_CLOCK_HPP
#define FINDTHEPATH_CLOCK_HPP

#include <chrono>
#include <iostream>

template<typename T>
concept Printable = requires(T t) {
    std::cout << t << std::endl;
};

class Clock {
public:
    Clock() = default;
    void print(Printable auto &msg) {
#if defined(__linux__)
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << msg << elapsed_seconds.count() << "s\n";
        start = end;
#endif
    }

private:
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> start = std::chrono::steady_clock::now();
};

#endif// FINDTHEPATH_CLOCK_HPP
