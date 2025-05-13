#include "LineTracking.hpp"

void LineTracking::FollowingLine( uint8_t Case, uint16_t LeftSpeed, uint16_t RightSpeed ){
  switch (Case)
  {
  case OnTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(1);
  break;

  case IR_LOnTrack:
    Motion::Leftwards(LeftSpeed, RightSpeed);
    delay(20);
  break;

  case IR_ROnTrack:
    Motion::Rightwards(LeftSpeed, RightSpeed);
    delay(15);
  break;

  case AllOnTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(2);
  break;
  
  case OutOfTrack:
    Motion::Forwards(LeftSpeed, RightSpeed);
    delay(2);

  break;


  }



};