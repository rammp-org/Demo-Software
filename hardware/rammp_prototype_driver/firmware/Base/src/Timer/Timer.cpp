#include "Timer.h"

// TODO: consider switching off of millis
void Timer::updateTime() {
  previous_time = current_time;
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000;
}
