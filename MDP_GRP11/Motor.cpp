#include "Motor.h"


Motor::Motor() {
  M1Ticks=0;
  M1Ticks=0;
  md.init();
}

void Motor::init() {
}

void Motor::encoder1() {
  M1Ticks++;
}

void Motor::encoder2() {
  M2Ticks++;
}

void Motor::resetTicks() {
  M1Ticks = 0;
  M2Ticks = 0;
}

long Motor::getM1Ticks() {
  return M1Ticks;
}

long Motor::getM2Ticks() {
  return M2Ticks;
}
