
#ifndef MotorControl_H_
#define MotorControl_H_
#include "esp32-hal-ledc.h"
#include "Pinout.hpp"
#include <Arduino.h>

#define RightWheel 1
#define LeftWheel  2

namespace Motor{
  void Init();
  void Moving_Clockwise(uint16_t Speed, uint8_t Wheel);
  void Moving_AntiClockwise(uint16_t Speed, uint8_t Wheel);
  void Stop();
  float RPMtoPWM(float TargetRPM, uint8_t Wheel);
};

namespace Motion{
  void Forwards(uint16_t LeftSpeed, uint16_t RightSpeed);
  void Backwards(uint16_t LeftSpeed, uint16_t RightSpeed);
  void Rightwards(uint16_t LeftSpeed, uint16_t RightSpeed);
  void Leftwards(uint16_t LeftSpeed, uint16_t RightSpeed);
};

namespace Servo {
  void Init();
  void TrunDeg(uint16_t Degree);
}
#endif