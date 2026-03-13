#include "Timer.h"

void Timer::updateTime() {
  previous_time = current_time;
  current_time = millis();
  elapsed_time = (current_time - previous_time) / 1000;
}
