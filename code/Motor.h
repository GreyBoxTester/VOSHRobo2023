#pragma once
#include <Arduino.h>

class Motor
{
public:
  Motor(uint8_t sidePin, uint8_t pwmPin)
    : sidePin(sidePin), pwmPin(pwmPin)
  {
    pinMode(sidePin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
  }
  
  void setPower(int16_t power)
  {
    if (power == 0)
    {
      digitalWrite(sidePin, LOW);
      digitalWrite(pwmPin, LOW);
    }
    else if (power > 0)
    {
      digitalWrite(sidePin, LOW);
      analogWrite(pwmPin, min(power, 255));
    }
    else
    {
      digitalWrite(sidePin, HIGH);
      analogWrite(pwmPin, 255 + max(power, -255));
    }
  }
  
private:
  uint8_t sidePin;
  uint8_t pwmPin;
};
