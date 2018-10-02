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
  while (1) { //i++ < 220){
    readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
    if (DEBUG_SENSOR) Serial.println("------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));
  }
}

void Sensor::readObstacle(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR) {
  int dFL = irDistance(1), dFM = irDistance(2), dFR = irDistance(3);
  int dLF = irDistance(5), dLB = irDistance(6);
  int dR = irDistance(4);

  *oFL = dFL < rFL[0] ? 0 : dFL < rFL[1] ? 1 : dFL < rFL[2] ? 2 : dFL < rFL[3] ? 3 : 9;
  *oFM = dFM < rFM[0] ? 0 : dFM < rFM[1] ? 1 : dFM < rFM[2] ? 2 : dFM < rFM[3] ? 3 : 9;
  *oFR = dFR < rFR[0] ? 0 : dFR < rFR[1] ? 1 : dFR < rFR[2] ? 2 : dFR < rFR[3] ? 3 : 9;

  *oLF = dLF < rLF[0] ? 0 : dLF < rLF[1] ? 1 : dLF < rLF[2] ? 2 : dLF < rLF[3] ? 3 : 9;
  *oLB = dLB < rLB[0] ? 0 : dLB < rLB[1] ? 1 : dLB < rLB[2] ? 2 : dLB < rLB[3] ? 3 : 9;

  *oR = dR < rR[0] ? 0 : dR < rR[1] ? 1 : dR < rR[2] ? 2 : dR < rR[3] ? 3 : dR < rR[4] ? 4 : dR < rR[5] ? 5 : 9;
}

void Sensor::readObstacle(int *oF, int *oL, int *oR) {

  //NEED TO READ THEM ONE BY ONE, ONLY WHEN NEEDED
  //Detect front sensors
  if (*oF == 0) {
    int dFL, dFM = irDistance(2), dFR;
    if (dFM < rFM[0]) {
      *oF = 0;
      if (DEBUG_SENSOR) Serial.println("F1");
    } else if (dFM > rFM[0]) {
      dFL = irDistance(1);
      if (DEBUG_SENSOR) Serial.println("F2");
      if (dFL < rFL[0]) {
        *oF = 0;
        if (DEBUG_SENSOR) Serial.println("F3");
      } else {
        dFR = irDistance(3);
        if (DEBUG_SENSOR) Serial.println("F4");

        if (dFR < rFR[0]) {
          *oF = 0;
          if (DEBUG_SENSOR) Serial.println("F5");
        }

        if (dFM < rFM[1] || dFL < rFL[1] || dFR < rFR[1]) {
          *oF = 1;
          if (DEBUG_SENSOR) Serial.println("F6");
        } else if (dFM < rFM[2] || dFL < rFL[2] || dFR < rFR[2]) {
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
    if (dLF < rLF[0]) {
      *oL = 0;
      if (DEBUG_SENSOR) Serial.println("L1");
    } else if (dLF > rLF[0]) {
      dLB = irDistance(6);
      if (DEBUG_SENSOR) Serial.println("L2");
      if (dLB < rLB[0]) {
        *oL = 0;
        if (DEBUG_SENSOR) Serial.println("L3");
      }

      if (dLF < rLF[1] || dLB < rLB[1]) {
        *oL = 1;
        if (DEBUG_SENSOR) Serial.println("L4");
      } else if (dLF < rLF[2] || dLB < rLB[2]) {
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
    if (dR < rR[0]) {
      *oR = 0;
      if (DEBUG_SENSOR) Serial.println("R1");
    } else if (dR < rR[1]) {
      *oR = 1;
      if (DEBUG_SENSOR) Serial.println("R2");
    } else if (dR < rR[2]) {
      *oR = 2;
      if (DEBUG_SENSOR) Serial.println("R3");
    } else if (dR < rR[3]) {
      *oR = 3;
      if (DEBUG_SENSOR) Serial.println("R4");
    } else if (dR < rR[4]) {
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
  }

  mergeSort(ir_val, 0, NB_SAMPLE - 1);

  //Return in MM
  //Integer overflow error
  //therefore will return -ve value...

  int result = sensor == 4 ? 603.74 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0, -1.16) : 277.28 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000) / 1000.0, -1.2045);
  result = result < 0 ? 32767 : result;
  if (DEBUG_SENSOR) Serial.print(" | " + String(sensor) + " | " + String(ir_val[NB_SAMPLE / 2]) + " | " + String(result));
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
