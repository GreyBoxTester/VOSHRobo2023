#pragma once
#include <avr/wdt.h>

enum class Timeout : uint8_t { T16ms = 0, T32ms, T64ms, T128ms, T256ms, T512ms, T1024ms, T2048ms, T4096ms, T8192ms };

class WatchdogTimer
{
public:
  static void enable(Timeout timeout)
  {
    asm volatile ("WDR");
    
    uint8_t sregCopy = SREG;
    cli();             
    WDTCSR = ((1 << WDCE) | (1 << WDE));    
    WDTCSR = (1 << WDIE) | (((uint8_t)timeout & 0b111) | (((uint8_t)timeout & 0b1000) << (WDP3 - 3)));
    SREG = sregCopy;
  }

  static void disable()
  {
    uint8_t sregCopy = SREG;
    cli();             
    WDTCSR = ((1 << WDCE) | (1 << WDE));    
    WDTCSR = 0;
    SREG = sregCopy;
  }

  static void reset()
  {
    asm volatile ("WDR");  
  }

  static void getRealTimeouts(uint16_t* timeouts)
  {
    interruptFlag = false;
    uint64_t us = micros();
    enable(Timeout::T16ms);
    while(!interruptFlag);
    us = micros() - us;
    disable();

    for(uint8_t i = 0; i < 10; i++)
    {
      timeouts[i] = (us << i) / 1000;
    }
  }
public:
  static volatile bool interruptFlag;
};

volatile bool WatchdogTimer::interruptFlag = false;

ISR(WDT_vect) 
{
   WatchdogTimer::interruptFlag = true;
}
