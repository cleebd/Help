#ifndef Sensor_H_
#define Sensor_H_
#include "Pinout.hpp"
#include <Arduino.h>

#define OutOfTrack  0 
#define OnTrack     1
#define IR_ROnTrack 2
#define IR_LOnTrack 3 
#define IR_Finish   4
#define AllOnTrack  5

/*define sound speed in m*/
#define SOUND_SPEED 340

namespace IR {
  void Init();
  uint8_t Tracking();

};

namespace Ultrasonic {
  void Init();
  float GetDistance(); //return distance in meter
}


#endif