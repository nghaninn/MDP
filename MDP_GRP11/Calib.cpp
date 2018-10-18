#include "Calib.h"
#include "Global.h"

int rFL, rFM, rFR, rLF, rLB, rR;
int oFL, oFM, oFR, oLF, oLB, oR;
int aFL[3], aFM[3], aFR[3], aLF[3], aLB[3], aR[3];
int bFL[3], bFM[3], bFR[3], bLF[3], bLB[3], bR[3];

Calib::Calib() {
  mov = new Movement();
  selfCalibNeeded = false;
  sensor = new Sensor();
  isCalibrating = false;
}

void Calib::needSelfCalib() {
  selfCalibNeeded = true;
}

void Calib::selfCalib() {
  selfCalib(true);
}

void Calib::calibDelay() {
  delay(100);
}

void Calib::selfCalib(bool multipleLevel) {
  isCalibrating = true;

  int calibLEVEL = sensor->hasObstacleForSelfCalib();

  if (calibLEVEL == 1) {
    if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 1");

    calibWallL();
  }

  isCalibrating = false;
}

void Calib::calibLeft() {
  isCalibrating = true;

  if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 3");

  int calibLEVEL = sensor->hasObstacleForSelfCalib();

  if (calibLEVEL == 2) {
    mov->rotateL(90);
    calibDelay();

    calibForward();
    calibDelay();

    calibWallF();
    calibDelay();

    mov->rotateR(90);
    calibDelay();

    calibWallL();
  }
  isCalibrating = false;
}

/*
   Can only be called by ALOGO
*/
void Calib::calib() {
  isCalibrating = true;

  if (DEBUG_CALIB) Serial.println("CALIB");

  int i = 0;

  while ((i++) < 1) {
    if (sensor->hasObstacleForCalib()) {

      if (DEBUG_CALIB) Serial.println("NEED CALIB");

      calibForward(false);
      calibDelay();

      sensor->readFrontSensorRawValues();
      calibWallF();
      calibDelay();

      mov->rotateL(90);
      calibDelay();

      sensor->readFrontSensorRawValues();
      calibForward(false);
      calibDelay();

      sensor->readFrontSensorRawValues();
      calibWallF();
      calibDelay();

      mov->rotate(90);
      calibDelay();

      sensor->readLeftSensorRawValues();
      calibWallL();
    }

    isCalibrating = false;
  }

  if (DEBUG_CALIB) sensor->readSensorRawValues();
  if (DEBUG_CALIB) Serial.println("ENDING SENSOR READING " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]));
}

void Calib::calibFront() {
  isCalibrating = true;

  if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 2");

  calibForward(false);
  calibDelay();

  calibWallF();
  isCalibrating = false;
}

void Calib::calibForward() {
  calibForward(true);
}

void Calib::calibForward(bool limitedFront) {
  int i = 100;
  int distance = 0;

  sensor->readSensorRawValues();
  if (DEBUG_CALIB) Serial.println("Calib Forward: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]));

  if (rFM >= rFL && rFM >= rFR && rFL < sFL_o[0] && rFM < sFM_o[0] && rFR < sFR_o[0]) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK (AT WALL)");
    distance = -40;
  } else if (rFM - sFM_o[0] > 2) {
    if (DEBUG_CALIB) Serial.println("MORE THAN 10mm");
    distance = (rFM - sFM_o[0]) * 3;
  } else if (rFM - sFM_o[0] < -2) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK");
    distance = -1;
  }

  if (abs(distance) > 0) {
    distance = (((abs(distance) > 50) && limitedFront) ? ((distance > 0 && limitedFront) ? 50 : (-1 * 50)) : ((abs(distance) > Distance_MAX) ? ((distance > 0) ? Distance_MAX : (-1 * 50)) : distance));
    mov->moveSmall(distance);
    if (DEBUG_CALIB) Serial.println("Calib Moving Forward: " + String(distance));
  }

  do {
    sensor->readSensorRawValues();

    if (DEBUG_CALIB) Serial.println("Calib Forward: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]));

    int distance = 0;

    if (rFM >= rFL && rFM >= rFR && rFL < sFL_o[0] && rFM < sFM_o[0] && rFR < sFR_o[0]) {
      if (DEBUG_CALIB) Serial.println("MOVE BACK (AT WALL)");
      distance = -1;
    } else if (rFM - sFM_o[0] > 2) {
      if (DEBUG_CALIB) Serial.println("MORE THAN 10mm");
      distance = 1;
    } else if (rFM - sFM_o[0] < -2) {
      if (DEBUG_CALIB) Serial.println("MOVE BACK");
      distance = -1;
    }

    if (abs(distance) > 0) {
      distance = (((abs(distance) > 50) && limitedFront) ? ((distance > 0 && limitedFront) ? 50 : (-1 * 50)) : ((abs(distance) > Distance_MAX) ? ((distance > 0) ? Distance_MAX : (-1 * 50)) : distance));
      mov->moveSmall(distance);
      if (DEBUG_CALIB) Serial.println("Calib Moving Forward: " + String(distance));
    }
  } while (abs(rFM - sFM_o[0]) > 1 && i--);
}

void Calib::calibWallF() {
  int i = 100;
  do {
    sensor->readFrontSensorRawValues();

    if (DEBUG_CALIB) Serial.println("Calib Wall F: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | DEFAULT: " + String(sFM[0]) );

    int distance = 0;
    int correspondingFL = map(map(rFR, sFR_Limit[0], sFR_Limit[1], 0, 5000), 0, 5000, sFL_Limit[0], sFL_Limit[1]);
    if (DEBUG_CALIB) Serial.println("Corresponding Distance: " + String(correspondingFL));

    if ((correspondingFL - rFL) < -2) { //rotate right
      distance = 1;
    } else if ((correspondingFL - rFL) > 2) { // rotate left
      distance = -1;
    }

    mov->rotateSmall(distance);
    if (DEBUG_CALIB) Serial.println("Calib F Rotating: " + String(distance));
  } while (abs(rFL - rFR) > 1 && i--);

  selfCalibNeeded = false;
}

void Calib::calibWallL() {
  sensor->readLeftSensorRawValues();
  if (DEBUG_CALIB) Serial.println("Calib Wall L: " + String(rLF) + " | " + String(rLB) );

  int distance = 0;

  if (sensor->hasObstacleForSelfCalib()) {

    int correspondingLB = map(map(rLF, sLF_Limit[0], sLF_Limit[1], 0, 5000), 0, 5000, sLB_Limit[0], sLB_Limit[1]);
    if (DEBUG_CALIB) Serial.println("Corresponding Distance: " + String(correspondingLB));

    if ((correspondingLB + LB_Calib_Offset - rLB) < -2) { //rotate right
      distance = (correspondingLB + LB_Calib_Offset - rLB) * -1;
    } else if ((correspondingLB + LB_Calib_Offset - rLB) > 2) { // rotate left
      distance = (correspondingLB + LB_Calib_Offset - rLB) * -1;
    }

    if (abs(distance) > 0) {
      distance = (abs(distance) > Rotate_45deg ? (distance > 0 ? Rotate_45deg : Rotate_45deg  - 1) : distance);
      if (DEBUG_CALIB) Serial.println("Moving Distance: " + String(distance));
      mov->rotateSmall(distance);
    }

    int i = 30;
    do {
      calibDelay();
      sensor->readLeftSensorRawValues();
      if (DEBUG_CALIB) Serial.println("Calib Wall L: " + String(rLF) + " | " + String(rLB) );

      distance = 0;
      correspondingLB = map(map(rLF, sLF_Limit[0], sLF_Limit[1], 0, 5000), 0, 5000, sLB_Limit[0], sLB_Limit[1]);
      if (DEBUG_CALIB) Serial.println("Corresponding Distance: " + String(correspondingLB));

      if ((correspondingLB + LB_Calib_Offset - rLB) < -2) { //rotate right
        distance = 1;
      } else if ((correspondingLB + LB_Calib_Offset - rLB) > 2) { // rotate left
        distance = -1;
      }

      mov->rotateSmall(distance);
      if (DEBUG_CALIB) Serial.println("Calib L Rotating: " + String(distance));

    } while (abs(correspondingLB + LB_Calib_Offset - rLB) && i--);

    selfCalibNeeded = false;
  }
}
