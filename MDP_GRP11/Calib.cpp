#include "Calib.h"

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
  int rLF, rLB;

  sensor->readLeftSensorRawValues(&rLF, &rLB);

  int calibLEVEL = sensor->hasObstacleForSelfCalib(&rLF, &rLB);

  if (calibLEVEL == 1) {
    if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 1");

    calibWallL(&rLF, &rLB);
  }
  isCalibrating = false;
}

void Calib::calibLeft() {
  isCalibrating = true;

  if (DEBUG_CALIB) Serial.println("SELF CALIB LEVEL 3");
  int rLF, rLB;

  sensor->readLeftSensorRawValues(&rLF, &rLB);

  int calibLEVEL = sensor->hasObstacleForSelfCalib(&rLF, &rLB);

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

  int rFL, rFM, rFR, rLF, rLB, rR, i = 0;

  while ((i++) < 2) {
    sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);

    if (DEBUG_CALIB) Serial.println(String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | " );

    if (sensor->hasObstacleForCalib(&rFL, &rFM, &rFR, &rLF, &rLB, &rR)) {
      if (DEBUG_CALIB) Serial.println("NEED CALIB");
      calibForward(&rFL, &rFM, &rFR, false);
      calibDelay();

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibWallF(&rFL, &rFM, &rFR);
      calibDelay();

      mov->rotateL(90);
      calibDelay();

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibForward(&rFL, &rFM, &rFR, false);
      calibDelay();

      sensor->readFrontSensorRawValues(&rFL, &rFM, &rFR);
      calibWallF(&rFL, &rFM, &rFR);
      calibDelay();

      mov->rotate(90);
      calibDelay();

      sensor->readLeftSensorRawValues(&rLF, &rLB);
      calibWallL(&rLF, &rLB);
    }
    //    calibWallL();
    isCalibrating = false;
  }

  if (DEBUG_CALIB) sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);
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
  int rFL, rFM, rFR, rLF, rLB, rR;
  sensor->readSensorRawValues(&rFL, &rFM, &rFR, &rLF, &rLB, &rR);

  if (DEBUG_CALIB) Serial.println("Calib Forward: " + String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | DEFAULT: " + String(sFM[0]));
  //  Serial.println("Calib Forward");

  calibForward(&rFL, &rFM, &rFR, limitedFront);
}


void Calib::calibForward(int *rFL, int *rFM, int *rFR, bool limitedFront) {
  int distance = 0;

  if (*rFM >= *rFL && *rFM >= *rFR && *rFL < sFL && *rFM < sFM && *rFR < sFR) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK (AT WALL)");
    distance = -130;
  } else if (*rFM - sFM_o[0] > 2) {
    if (DEBUG_CALIB) Serial.println("MORE THAN 10mm");
    distance = (*rFM - sFM_o[0]) * 3;
  } else if (*rFM - sFM_o[0] < -2) {
    if (DEBUG_CALIB) Serial.println("MOVE BACK");
    distance = abs(*rFM - sFM_o[0]) * -3;;
  }

  if (abs(distance) > 0) {
    distance = (((abs(distance) > Distance_5CM) && limitedFront) ? ((distance > 0 && limitedFront) ? Distance_5CM : (-1 * Distance_5CM)) : ((abs(distance) > Distance_MAX) ? ((distance > 0) ? Distance_MAX : (-1 * Distance_5CM)) : distance));
    mov->moveSmall(distance);
    if (DEBUG_CALIB) Serial.println("Calib Moving Forward: " + String(distance));
  } if (distance < -100)
    calibForward();
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
    distance = (abs(distance) > Rotate_45deg ? (distance > 0 ? Rotate_45deg : Rotate_45deg * -1) : distance);
    mov->rotateSmall(distance);
    if (DEBUG_CALIB) Serial.println("Calib F Rotating: " + String(distance));
  }

  selfCalibNeeded = false;
}

void Calib::calibWallL() {

  int rLF, rLB;
  sensor->readLeftSensorRawValues(&rLF, &rLB);

  if (DEBUG_CALIB) Serial.println("Calib Wall L: " + String(rLF) + " | " + String(rLB) );

  calibWallL(&rLF, &rLB);
}

void Calib::calibWallL(int *rLF, int *rLB) {

  if (sensor->hasObstacleForSelfCalib(rLF, rLB)) {

    do {
      calibDelay();

      int distance = 0;

      //      if ((*rLB + LB_Calib_Offset - *rLF) > 2) { //rotate right
      //        if (*rLB + LB_Calib_Offset - *rLF > 6)
      //          distance = (*rLB + LB_Calib_Offset - *rLF) * 1;
      //        else
      //          distance = 5;
      //      } else if ((*rLB + LB_Calib_Offset - *rLF) < -2) { // rotate left
      //        if (*rLB + LB_Calib_Offset - *rLF < -6)
      //          distance = (*rLB + LB_Calib_Offset - *rLF) * 1;
      //        else
      //          distance = 5;
      //      }

      if ((*rLB + LB_Calib_Offset - *rLF) > 2) { //rotate right
        //        if (*rLB + LB_Calib_Offset - *rLF > 6)
        distance = (*rLB + LB_Calib_Offset - *rLF) * 1;
        //        else
        //          distance = 5;
      } else if ((*rLB + LB_Calib_Offset - *rLF) < -2) { // rotate left
        //        if (*rLB + LB_Calib_Offset - *rLF < -6)
        distance = (*rLB + LB_Calib_Offset - *rLF) * 1;
        //        else
        //          distance = 5;
      }

      if (abs(distance) > 0) {
        distance = (abs(distance) > Rotate_45deg ? (distance > 0 ? Rotate_45deg : Rotate_45deg * -1) : distance);

        mov->rotateSmall(distance);
        if (DEBUG_CALIB) Serial.println("Calib L Rotating: " + String(distance));
      }

      sensor->readLeftSensorRawValues(rLF, rLB);
    } while (0); //while (abs(*rLB + LB_Calib_Offset - *rLF));

    selfCalibNeeded = false;
  }
}
