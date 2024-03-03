#include "competition/autonomous.h"
#include "robot-config.h"

void just_auto();
void autonomous() {
  while (imu.isCalibrating()) {
    vexDelay(1);
  }
}

void just_auto() {
  CommandController cc{
    odom.SetPositionCmd({.x = 0, .y = 0, .rot = 90}),

  };
}