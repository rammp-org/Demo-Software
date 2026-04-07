#include "Timer.h"
#include <Arduino.h>

// TODO: consider switching off of millis
void Timer::updateTime() {
  previous_time = current_time;
  current_time = micros();
  elapsed_time = (current_time - previous_time) / 1000000.0f;
}
