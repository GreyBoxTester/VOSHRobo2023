#pragma once
#include <avr/sleep.h>
#include <Arduino.h>
#include "WatchdogTimer.h"

#define PWR_ALL   0b11101111
#define PWR_TWI   0b10000000
#define PWR_TIM2  0b01000000
#define PWR_TIM0  0b00100000
#define PWR_TIM1  0b00001000
#define PWR_SPI   0b00000100
#define PWR_UART0 0b00000010
#define PWR_ADC   0b00000001

#define REAL_1V1_REF_VOLTAGE 1100

enum class Prescaler : uint8_t { X1 = 0, X2, X4, X8, X16, X32, X64, X128, X256 };

class Microcontroller
{
public:
  Microcontroller() {}

  void calibrateWDTTimings()
  {
    WatchdogTimer::getRealTimeouts(wdtTimeouts);
  }

  void enablePeripherals(uint8_t peripherals)
  {
    PRR &= ~peripherals;

    if (peripherals & PWR_ADC) 
    {
      ADCSRA |= (1 << ADEN);
      ACSR &= ~(1 << ACD);
    }
  }
  
  void disablePeripherals(uint8_t peripherals)
  {
    if (peripherals & PWR_ADC) 
    {
      ADCSRA &= ~(1 << ADEN);
      ACSR |= (1 << ACD);
    }
    
    PRR |= peripherals;
  }

  void sleep(uint32_t timeout)
  {
    uint8_t enabledPeripherals = ~PRR;
    disablePeripherals(enabledPeripherals);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
  
    for(int8_t i = 9; i > -1; i--)
    {
      while(timeout > wdtTimeouts[i])
      {
        timeout -= wdtTimeouts[i];

        WatchdogTimer::enable((Timeout)i);
        sleep_cpu();
        WatchdogTimer::disable();
      }
    }
    
    sleep_disable();

    enablePeripherals(enabledPeripherals);
  }
  
  void sleep(Timeout timeout)
  {
    uint8_t enabledPeripherals = ~PRR;
    disablePeripherals(enabledPeripherals);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    WatchdogTimer::enable(timeout);
    sleep_cpu();
    WatchdogTimer::disable();
    sleep_disable();

    enablePeripherals(enabledPeripherals);
  }

  void sleep()
  {
    uint8_t enabledPeripherals = ~PRR;
    disablePeripherals(enabledPeripherals);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();

    enablePeripherals(enabledPeripherals);
  }

  void setClockPrescaler(Prescaler prescaler)
  {
    CLKPR = (1 << CLKPCE);
    CLKPR = (uint8_t)prescaler;
  }
  
  uint16_t readPowerSupplyVoltage()
  {
    ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1);

    delayMicroseconds(350);
    
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    return REAL_1V1_REF_VOLTAGE * 1024UL / ADC;
  }

  uint16_t analogRead(uint8_t pin, uint8_t count)
  {
    uint16_t res = 0;
    for (uint8_t i = 0; i < count; i++) { res += ::analogRead(pin); }
    return res / count;
  }

  void connectADCToGround()
  {
    ADMUX = (1 << REFS0) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0);
    delayMicroseconds(350);
  }
  
private:
  uint16_t wdtTimeouts[10] = { 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
};
