// Copyright 2021 Feiler

#include "Timer.h"

Timer::Timer(int seconds) : itsSeconds(seconds) {
    reset();
}

void Timer::reset() {
    resetTimePoint = std::chrono::high_resolution_clock::now();
}

bool Timer::expired() {
    bool timerExpired = false;
    std::chrono::seconds timeDiffInSec = std::chrono::duration_cast<std::chrono::seconds>
        (std::chrono::high_resolution_clock::now() - resetTimePoint);
    if ( timeDiffInSec.count() > itsSeconds ) {
        timerExpired = true;
    }
    return timerExpired;
}
