#include <Arduino.h>
#include <EEPROM.h>
#include "Motor.h"
#include "Microcontroller.h"
#include "Timer.h"

//====== settings ======

#define SLEEP_TIMEOUT 512                             //in ms
#define ACTIVATION_BRIGHTNESS 37                      //difference between zero brightness and activation brightness
#define OBSTACLE_BRIGHTNESS 500                       //difference between zero brightness and obstacle brightness
#define BRIGHTNESS_MEASURE_ACCURACY 40                //larger number makes better accuracy but increases measure time 
#define WATER_FLOW_OPENING_SPEED -255                 //-255 - 255
#define WATER_FLOW_CLOSING_SPEED 255                  //-255 - 255
#define DEFAULT_WATER_FLOW_OPENING_CLOSING_TIME 700   //in ms
#define STARTUP_WAITING_TIME 7000                     //in ms
#define SETUP_WAITING_TIME 3000                       //in ms
#define DISCHARGED_BATTERY_VOLTAGE 3000               //in mV

//===== pins =====

#define IR_SENSOR_PIN A0
#define IR_SENSOR_PIN_EN 7
#define RED_LED_PIN 9
#define MOTOR_SIDE_PIN 4
#define MOTOR_PWM_PIN 5

//====== main code ======

Microcontroller mcu;
Motor motor(MOTOR_SIDE_PIN, MOTOR_PWM_PIN);
uint16_t activationBrightness;
uint16_t obstacleBrightness;
Timer timer;
volatile uint32_t waterFlowOpeningClosingTime;
bool waterFlowOpened = false;


void openWaterFlow()
{
  waterFlowOpened = true;
  motor.setPower(WATER_FLOW_OPENING_SPEED);
  delay(waterFlowOpeningClosingTime);
  motor.setPower(0);
}

void closeWaterFlow()
{
  waterFlowOpened = false;
  motor.setPower(WATER_FLOW_CLOSING_SPEED);
  delay(waterFlowOpeningClosingTime);
  motor.setPower(0);
}

void blinkLED(uint8_t times)
{
  for(; times > 0; times--)
  {
    digitalWrite(RED_LED_PIN, HIGH);
    mcu.sleep(Timeout::T64ms);
    digitalWrite(RED_LED_PIN, LOW);
    mcu.sleep(Timeout::T64ms);
  }
}

uint16_t readIRSensor()
{
  digitalWrite(IR_SENSOR_PIN_EN, LOW);
  delay(1);
  uint16_t res = 1023 - mcu.analogRead(IR_SENSOR_PIN, BRIGHTNESS_MEASURE_ACCURACY);
  digitalWrite(IR_SENSOR_PIN_EN, HIGH);
  return res;
}

void setup()
{
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(IR_SENSOR_PIN_EN, OUTPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  digitalWrite(IR_SENSOR_PIN_EN, HIGH);

  mcu.sleep(1024);

  mcu.disablePeripherals(PWR_TWI | PWR_TIM1 | PWR_TIM2 | PWR_SPI | PWR_UART0);
  
  activationBrightness = obstacleBrightness = readIRSensor();
  activationBrightness += ACTIVATION_BRIGHTNESS;
  obstacleBrightness += OBSTACLE_BRIGHTNESS;
  
  EEPROM.get(0, waterFlowOpeningClosingTime);
  if (waterFlowOpeningClosingTime == 0xffffffff) { waterFlowOpeningClosingTime = DEFAULT_WATER_FLOW_OPENING_CLOSING_TIME; }

  //close water flow if hands are near
  digitalWrite(RED_LED_PIN, 1);
  timer.reset();
  while(timer.getElapsedTime() < STARTUP_WAITING_TIME)
  {
    if (readIRSensor() >= obstacleBrightness)
    {
      motor.setPower(WATER_FLOW_CLOSING_SPEED);
      while (readIRSensor() >= obstacleBrightness);
      motor.setPower(0);
      break;
    }
  }
  digitalWrite(RED_LED_PIN, 0);
  
  blinkLED(2);
}

void loop()
{ 
  //if battery is discharged then sleep forever
  if(mcu.readPowerSupplyVoltage() <= DISCHARGED_BATTERY_VOLTAGE)
  {
    if (waterFlowOpened) { closeWaterFlow(); }
    digitalWrite(IR_SENSOR_PIN_EN, HIGH);
    mcu.sleep();
  }

  uint16_t brightness = readIRSensor();

  if (brightness >= obstacleBrightness && !waterFlowOpened)
  {
    timer.reset();
    while (readIRSensor() >= obstacleBrightness && timer.getElapsedTime() < SETUP_WAITING_TIME);
    if (timer.getElapsedTime() < SETUP_WAITING_TIME) { return; }

    //setup opening closing time
    timer.reset();
    motor.setPower(WATER_FLOW_OPENING_SPEED);
    while (readIRSensor() >= obstacleBrightness);
    waterFlowOpeningClosingTime = timer.getElapsedTime();
    motor.setPower(0);
    EEPROM.put(0, waterFlowOpeningClosingTime);
    closeWaterFlow();
    return;
  }
  
  bool isInRange = (brightness >= activationBrightness && brightness < obstacleBrightness);
  if (isInRange && !waterFlowOpened)
  {
    blinkLED(2);
    openWaterFlow();
  }
  else if (!isInRange && waterFlowOpened)
  {
    blinkLED(1);
    closeWaterFlow();
  }
  
  mcu.sleep(SLEEP_TIMEOUT);
}
