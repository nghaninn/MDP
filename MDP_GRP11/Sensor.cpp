#include "Sensor.h"

Sensor::Sensor() {
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  pinMode (ir4, INPUT);
  pinMode (ir5, INPUT);
  pinMode (ir6, INPUT);
  analogReference(DEFAULT);
}

void Sensor::detect() {
  int oF, oL, oR;
  while (1) {
    readObstacle(&(oF = 0), &(oL = 0), &(oR = 0));
    if (DEBUG_SENSOR) Serial.println("------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
  }
}

void Sensor::detectAll() {
  int oFL, oFM, oFR, oLF, oLB, oR;
  int i = 0;
  while (i++ < 2200) {//1) { //
    readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
    //    if (DEBUG_SENSOR) Serial.println("------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));
    Serial.println("");
  }
}

void Sensor::readObstacle(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR) {
  int dFL = irDistance(1), dFM = irDistance(2), dFR = irDistance(3);
  int dLF = irDistance(5), dLB = irDistance(6);
  int dR = irDistance(4);

  *oFL = dFL < sFL[0] ? gShort[0] : dFL < sFL[1] ? gShort[1] : dFL < sFL[2] ? gShort[2] : dFL < sFL[3] ? gShort[3] : gShort[4];
  *oFM = dFM < sFM[0] ? gShort[0] : dFM < sFM[1] ? gShort[1] : dFM < sFM[2] ? gShort[2] : dFM < sFM[3] ? gShort[3] : gShort[4];
  *oFR = dFR < sFR[0] ? gShort[0] : dFR < sFR[1] ? gShort[1] : dFR < sFR[2] ? gShort[2] : dFR < sFR[3] ? gShort[3] : gShort[4];

  *oLF = dLF < sLF[0] ? gShort[0] : dLF < sLF[1] ? gShort[1] : dLF < sLF[2] ? gShort[2] : dLF < sLF[3] ? gShort[3] : gShort[4];
  *oLB = dLB < sLB[0] ? gShort[0] : dLB < sLB[1] ? gShort[1] : dLB < sLB[2] ? gShort[2] : dLB < sLB[3] ? gShort[3] : gShort[4];

  *oR = dR < sR[0] ? gLong[0] : dR < sR[1] ? gLong[1] : dR < sR[2] ? gLong[2] : dR < sR[3] ? gLong[3] : dR < sR[4] ? gLong[4] : dR < sR[5] ? gLong[5] : gLong[6];
}

void Sensor::readSensorRawValues(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR) {
  readFrontSensorRawValues(oFL, oFM, oFR);
  readLeftSensorRawValues(oLF, oLB);
  readRightSensorRawValues(oR);
}

void Sensor::readFrontSensorRawValues(int *oFL, int *oFM, int *oFR) {
  *oFL = irDistance(1);
  *oFM = irDistance(2);
  *oFR = irDistance(3);
}

void Sensor::readLeftSensorRawValues(int *oLF, int *oLB) {
  *oLF = irDistance(5);
  *oLB = irDistance(6);
}

void Sensor::readRightSensorRawValues(int *oR) {
  *oR = irDistance(4);
}

bool Sensor::hasObstacleForCalib(int *rFL, int *rFM, int *rFR, int *rLF, int *rLB, int *rR) {

  int oFL = *rFL < sFL[0] ? gShort[0] : *rFL < sFL[1] ? gShort[1] : *rFL < sFL[2] ? gShort[2] : *rFL < sFL[3] ? gShort[3] : gShort[4];
  int oFM = *rFM < sFM[0] ? gShort[0] : *rFM < sFM[1] ? gShort[1] : *rFM < sFM[2] ? gShort[2] : *rFM < sFM[3] ? gShort[3] : gShort[4];
  int oFR = *rFR < sFR[0] ? gShort[0] : *rFR < sFR[1] ? gShort[1] : *rFR < sFR[2] ? gShort[2] : *rFR < sFR[3] ? gShort[3] : gShort[4];

  int oLF = *rLF < sLF[0] ? gShort[0] : *rLF < sLF[1] ? gShort[1] : *rLF < sLF[2] ? gShort[2] : *rLF < sLF[3] ? gShort[3] : gShort[4];
  int oLB = *rLB < sLB[0] ? gShort[0] : *rLB < sLB[1] ? gShort[1] : *rLB < sLB[2] ? gShort[2] : *rLB < sLB[3] ? gShort[3] : gShort[4];

  int oR = *rR < sR[0] ? gShort[0] : *rR < sR[1] ? gShort[1] : gShort[4];

  if (DEBUG_SENSOR) Serial.println("hasObsForCal: " + String(oFL) + " | " + String(oFM) + " | " + String(oFR) + " | " + String(oLF) + " | " + String(oLB) + " | " + String(oR));

  if ((oFL <= gShort[3] || oFM <= gShort[3] || oFR <= gShort[3]) && (oLF <= gShort[3] && oLB <= gShort[3]))
    return true;

  return false;
}

int Sensor::hasObstacleForSelfCalib(int *rLF, int *rLB) {

  int oLF = *rLF < sLF[0] ? gShort[0] : *rLF < sLF[1] ? gShort[1] : *rLF < sLF[2] ? gShort[2] : *rLF < sLF[3] ? gShort[3] : gShort[4];
  int oLB = *rLB < sLB[0] ? gShort[0] : *rLB < sLB[1] ? gShort[1] : *rLB < sLB[2] ? gShort[2] : *rLB < sLB[3] ? gShort[3] : gShort[4];

  if (DEBUG_SENSOR) Serial.println("hasObsForSelfCal: " + String(oLF) + " | " + String(oLB));

  if(oLF != oLB)
    return 0;

  if ((oLF < gShort[2] && *rLF > sLF_Limit[1]) || (oLB < gShort[2] && *rLB > sLB_Limit[1]) ||
      (oLF == gShort[0] && *rLF < sLF_Limit[0]) || (oLB == gShort[0] && *rLB < sLB_Limit[0]))
    return 2;
  else if ((*rLF < (sLF[1] * 0.75)) && (*rLB < (sLB[1] * 0.75)))
    return 1;

  return 0;
}

void Sensor::readObstacle(int *oF, int *oL, int *oR) {

  //NEED TO READ THEM ONE BY ONE, ONLY WHEN NEEDED
  //Detect front sensors
  if (*oF == 0) {
    int dFL, dFM = irDistance(2), dFR;
    if (dFM < sFM[0]) {
      *oF = 0;
      if (DEBUG_SENSOR) Serial.println("F1");
    } else if (dFM > sFM[0]) {
      dFL = irDistance(1);
      if (DEBUG_SENSOR) Serial.println("F2");
      if (dFL < sFL[0]) {
        *oF = 0;
        if (DEBUG_SENSOR) Serial.println("F3");
      } else {
        dFR = irDistance(3);
        if (DEBUG_SENSOR) Serial.println("F4");

        if (dFR < sFR[0]) {
          *oF = 0;
          if (DEBUG_SENSOR) Serial.println("F5");
        }

        if (dFM < sFM[1] || dFL < sFL[1] || dFR < sFR[1]) {
          *oF = 1;
          if (DEBUG_SENSOR) Serial.println("F6");
        } else if (dFM < sFM[2] || dFL < sFL[2] || dFR < sFR[2]) {
          *oF = 2;
          if (DEBUG_SENSOR) Serial.println("F7");
        } else {
          *oF = 3;
          if (DEBUG_SENSOR) Serial.println("F8");
        }
      }
    }
  }

  //Detect left sensors
  if (*oL == 0) {
    int dLF = irDistance(5), dLB;
    if (dLF < sLF[0]) {
      *oL = 0;
      if (DEBUG_SENSOR) Serial.println("L1");
    } else if (dLF > sLF[0]) {
      dLB = irDistance(6);
      if (DEBUG_SENSOR) Serial.println("L2");
      if (dLB < sLB[0]) {
        *oL = 0;
        if (DEBUG_SENSOR) Serial.println("L3");
      }

      if (dLF < sLF[1] || dLB < sLB[1]) {
        *oL = 1;
        if (DEBUG_SENSOR) Serial.println("L4");
      } else if (dLF < sLF[2] || dLB < sLB[2]) {
        *oL = 2;
        if (DEBUG_SENSOR) Serial.println("L5");
      } else {
        *oL = 3;
        if (DEBUG_SENSOR) Serial.println("L6");
      }
    }
  }

  if (*oR == 0) {
    int dR = irDistance(4);
    if (dR < sR[0]) {
      *oR = 0;
      if (DEBUG_SENSOR) Serial.println("R1");
    } else if (dR < sR[1]) {
      *oR = 1;
      if (DEBUG_SENSOR) Serial.println("R2");
    } else if (dR < sR[2]) {
      *oR = 2;
      if (DEBUG_SENSOR) Serial.println("R3");
    } else if (dR < sR[3]) {
      *oR = 3;
      if (DEBUG_SENSOR) Serial.println("R4");
    } else if (dR < sR[4]) {
      *oR = 4;
      if (DEBUG_SENSOR) Serial.println("R5");
    } else {
      *oR = 5;
      if (DEBUG_SENSOR) Serial.println("R6");
    }
  }
}

int Sensor::irDistance(int sensor) {
  for (int i = 0; i < NB_SAMPLE; i++) {
    ir_val[i] = analogRead(sensor == 1 ? ir1 : sensor == 2 ? ir2 : sensor == 3 ? ir3 : sensor == 4 ? ir4 : sensor == 5 ? ir5 : sensor == 6 ? ir6 : 0);
//    if(DEBUG_SENSOR) Serial.println("RAW Sensor: " + String(ir_val[i]));
  }

  mergeSort(ir_val, 0, NB_SAMPLE - 1);

  //Return in MM
  //Integer overflow error
  //therefore will return -ve value...

  int result = sensor == 4 ? 603.74 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0, -1.16) : 277.28 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0, -1.2045);
  result = result < 0 ? 32767 : result;
  if (DEBUG_SENSOR) Serial.print(" | " + String(sensor) + " | " + String(ir_val[NB_SAMPLE / 2]) + "_ | " + String(result));
  return result;
  //Return in mV
  //  return ir_val[NB_SAMPLE / 2];
}

void Sensor::merge(int arr[], int l, int m, int r) {
  int i, j, k;
  int n1 = m - l + 1;
  int n2 =  r - m;

  /* create temp arrays */
  int L[n1], R[n2];

  /* Copy data to temp arrays L[] and R[] */
  for (i = 0; i < n1; i++)
    L[i] = arr[l + i];
  for (j = 0; j < n2; j++)
    R[j] = arr[m + 1 + j];

  /* Merge the temp arrays back into arr[l..r]*/
  i = 0; // Initial index of first subarray
  j = 0; // Initial index of second subarray
  k = l; // Initial index of merged subarray
  while (i < n1 && j < n2) {
    if (L[i] <= R[j]) {
      arr[k] = L[i];
      i++;
    } else {
      arr[k] = R[j];
      j++;
    }
    k++;
  }

  /* Copy the remaining elements of L[], if there
    are any */
  while (i < n1) {
    arr[k] = L[i];
    i++;
    k++;
  }

  /* Copy the remaining elements of R[], if there
    are any */
  while (j < n2) {
    arr[k] = R[j];
    j++;
    k++;
  }
}

/* l is for left index and r is right index of the
   sub-array of arr to be sorted */
void Sensor::mergeSort(int arr[], int l, int r) {
  if (l < r) {
    // Same as (l+r)/2, but avoids overflow for
    // large l and h
    int m = l + (r - l) / 2;

    // Sort first and second halves
    mergeSort(arr, l, m);
    mergeSort(arr, m + 1, r);

    merge(arr, l, m, r);
  }
}

void Sensor::printAllSensors() {
  int oFL, oFM, oFR, oLF, oLB, oR;
  readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));

  Serial.println("a" + String(oLF) + "," + String(oLB) + "," + String(oFL) + "," + String(oFR) + "," + String(oFM) + "," + String(oR));
}

void Sensor::printAllSensorsRAW() {
  int oFL, oFM, oFR, oLF, oLB, oR;
  readSensorRawValues(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));

  Serial.println("a" + String(oLF) + "," + String(oLB) + "," + String(oFL) + "," + String(oFR) + "," + String(oFM) + "," + String(oR));
}
//
//void Sensor::printAllSensorsRAWCalib() {
//
//  while
//  int dFL = irDistance(1), dFM = irDistance(2), dFR = irDistance(3);
//  int dLF = irDistance(5), dLB = irDistance(6);
//  int dR = irDistance(4);
//
//  Serial.println("");
//}
