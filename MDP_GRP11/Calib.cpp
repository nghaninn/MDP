#include "Calib.h"

Calib::Calib() {
  mov = new Movement();
  selfCalibNeeded = false;
  sensor = new Sensor();
}

void Calib::needSelfCalib() {
  selfCalibNeeded = true;
}

void Calib::selfCalib() {
  calibWallL();
  selfCalibNeeded = false;
}

void Calib::calib() {
  int rFL, rFM, rFR, rLF, rLB, rR, i = 0;

  while ((i++) < 2) {
    sensor->readSensor(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);

    if (sensor->hasObstacleForCalib(&rFL, &rFM, &rFR, &rLF, &rLB, &rR)) {

      calibForward(&rFL, &rFM, &rFR);
      calibWallF(&rFL, &rFR);
      //    calibWallL();
    }
  }
}

void Calib::calibForward(int *rFL, int *rFM, int *rFR) {
  mov->rotateL(90);
  if (*rFM - sFM[0] > 30) {
    mov->moveSmall(30);
    delay(30);
  } else if (*rFM - sFM[0] < 0) {
    mov->moveSmall(-30);
    delay(10);
  } else if (*rFM - sFM[0] < 10) {
    mov->moveSmall(30);
    delay(10);
  } else {
    mov->moveSmall(30);
    delay(10);
  }
}
void Calib::calibWallF(int *rFL, int *rFR) {
  if (*rFL - *rFR > 30) {
    if (*rFL > *rFR) { //rotate right
      mov->rotateSmall(30);
    } else {
      mov->rotateSmall(-30);
    }
    delay(30);
  } else if (*rFL - *rFR < 10) {
    if (*rFL > *rFR) { //rotate right
      mov->rotateSmall(30);
    } else {
      mov->rotateSmall(-30);
    }
    delay(10);
  } else {
    if (*rFL > *rFR) { //rotate right
      mov->rotateSmall(30);
    } else {
      mov->rotateSmall(-30);
    }
    delay(20);
  }
}

void Calib::calibWallL() {

}
