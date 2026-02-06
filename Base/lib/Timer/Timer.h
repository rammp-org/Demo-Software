#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer {
   public:
    int counter = 0;
    float current_time = 0.0, elapsed_time = 0.0, previous_time = 0.0;

    void updateTime();
};

#endif