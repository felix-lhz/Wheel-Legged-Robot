#ifndef MY_TIMER_H
#define MY_TIMER_H

#include "Arduino.h"

class MyTimer {
private:
  int start_time;
public:
  MyTimer();
  MyTimer(double);
  ~MyTimer() {}
  void reset();
  int getTimeMicroSecond() const;
  double getTimeMilliSecond() const;
};

#endif