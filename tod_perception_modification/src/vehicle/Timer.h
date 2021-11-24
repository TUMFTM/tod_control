// Copyright 2021 Feiler

#pragma once
#include <chrono>

class Timer {
public:
    explicit Timer(int seconds);
    void reset();
    bool expired();

private:
    int itsSeconds;
    std::chrono::high_resolution_clock::time_point resetTimePoint;
};
