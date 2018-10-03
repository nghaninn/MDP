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
  int rLF, rLB;

  sensor->readLeftSensorRawValues(&rLF, &rLB);

  int calibLEVEL = sensor->hasObstacleForSelfCalib(&rLF, &rLB);

  if (calibLEVEL == 1) {
    if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 1");

    calibWallL(&rLF, &rLB);
  } else if (calibLEVEL == 2) {
    if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 2");

    mov->rotateL(90);
    calibForward();
    calibWallF();
    mov->rotateR(90);
    calibWallL();
  }
}

void Calib::calib() {
  if (DEBUG_CALIB) Serial.println("CALIB");
  
  int rFL, rFM, rFR, rLF, rLB, rR, i = 0;

  while ((i++) < 2) {
    sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);

    if (DEBUG_CALIB) Serial.println(String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | " );

    if (sensor->hasObstacleForCalib(&rFL, &rFM, &rFR, &rLF, &rLB, &rR)) {
      if (DEBUG_CALIB) Serial.println("NEED CALIB");
      calibForward(&rFL, &rFM, &rFR);

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibWallF(&rFL, &rFM, &rFR);

      mov->rotateL(90);

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibForward(&rFL, &rFM, &rFR);

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibWallF(&rFL, &rFM, &rFR);

      mov->rotate(90);

      sensor->readLeftSensorRawValues(&rLF, &rLB);
      calibWallL(&rLF, &rLB);
    }
    //    calibWallL();
  }

  if (DEBUG_CALIB) sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);
  if (DEBUG_CALIB) Serial.println("ENDING SENSOR READING " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]));
}

void Calib::calibForward() {
  int rFL, rFM, rFR, rLF, rLB, rR;
  sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);

  if (DEBUG_CALIB) Serial.println("Calib Forward: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]) );

  calibForward(&rFL, &rFM, &rFR);
}

void Calib::calibForward(int *rFL, int *rFM, int *rFR) {
  int distance = 0;

  if (*rFM >= *rFL || *rFM >= *rFR) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK (AT WALL)");
    distance = -130;
  } else if (*rFM - sFM[0] > 2) {
    if (DEBUG_CALIB) Serial.println("MORE THAN 10mm");
    distance = (*rFM - sFM[0]) * 3;
  } else if (*rFM - sFM[0] < -2) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK");
    distance = abs(*rFM - sFM[0]) * -3;;
  }

  if (abs(distance) > 0) {
    mov->moveSmall(distance);
    Serial.println("Calib Moving Forward: " + String(distance));
  } if (distance < -100)
    calibForward();

  delay(10);
}

void Calib::calibWallF() {

  int rFL, rFM, rFR;
  sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);

  if (DEBUG_CALIB) Serial.println("Calib Wall F: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | DEFAULT: " + String(sFM[0]) );

  calibWallF(&rFL, &rFM, &rFR);
}

void Calib::calibWallF(int *rFL, int *rFM, int *rFR) {
  int distance = 0;

  if ((*rFL - *rFR) > 2) { //rotate right
    if (*rFL - *rFR > 6)
      distance = (*rFL - *rFR) * 2;
    else
      distance = 5;
  } else if ((*rFL - *rFR) < -2) { // rotate left
    if (*rFL - *rFR < -6)
      distance = (*rFL - *rFR) * 2;
    else
      distance = 5;
  }

  if (abs(distance) > 0) {
    mov->rotateSmall(distance);
    Serial.println("Calib F Rotating: " + String(distance));
  }

  selfCalibNeeded = false;
  delay(10);
}

void Calib::calibWallL() {

  int rLF, rLB;
  sensor->readLeftSensorRawValues(&rLF, &rLB);

  if (DEBUG_CALIB) Serial.println("Calib Wall L: " + String(rLF) + " | " + String(rLB) + " | " + String(sFM[0]) );

  calibWallL(&rLF, &rLB);
}

void Calib::calibWallL(int *rLF, int *rLB) {

  if (sensor->hasObstacleForSelfCalib(rLF, rLB)) {

    int distance = 0;

    if ((*rLB + LB_Calib_Offset - *rLF) > 2) { //rotate right
      if (*rLB + LB_Calib_Offset - *rLF > 6)
        distance = (*rLB + LB_Calib_Offset - *rLF) * 1.5;
      else
        distance = 5;
    } else if ((*rLB + LB_Calib_Offset - *rLF) < -2) { // rotate left
      if (*rLB + LB_Calib_Offset - *rLF < -6)
        distance = (*rLB + LB_Calib_Offset - *rLF) * 1.5;
      else
        distance = 5;
    }

    if (abs(distance) > 0) {
      mov->rotateSmall(distance);
      Serial.println("Calib L Rotating: " + String(distance));
    }

    selfCalibNeeded = false;
  }
  delay(10);
}
