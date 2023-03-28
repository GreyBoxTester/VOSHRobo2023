#pragma once
#include <Arduino.h>

class Timer
{
public:
  Timer() : prevTime(millis()) {}
  
  uint32_t getElapsedTime() const
  {
    return millis() - prevTime;
  }

  void reset()
  {
    prevTime = millis();
  }
  
private:
  uint32_t prevTime;
};
