#include "esp32-hal.h"
#include "my_timer.h"

MyTimer::MyTimer() {
  start_time = micros();
}

MyTimer::MyTimer(double init) {
  start_time = micros() + init * 1000;
}

void MyTimer::reset() {
  start_time = micros();
}

int MyTimer::getTimeMicroSecond() const {
  return micros() - start_time;
}

double MyTimer::getTimeMilliSecond() const {
  return (micros() - start_time) / 1000.0;
}