#include "Sensor.h"
#include "Global.h"

Sensor::Sensor() {
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  pinMode (ir4, INPUT);
  pinMode (ir5, INPUT);
  pinMode (ir6, INPUT);
  analogReference(DEFAULT);

  if (SENSOR_STABILITY_TEST)
    NB_SAMPLE = 7;//5
}

//void Sensor::detect() {
//  int oF, oL, oR;
//  while (1) {
//    readObstacle();
//    if (DEBUG_SENSOR) Serial.println("------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
//  }
//}

void Sensor::detectAll() {
  //  int oFL, oFM, oFR, oLF, oLB, oR;
  int i = 0;
  while (i++ < 2200) {//1) { //
    readObstacle();
    //    if (DEBUG_SENSOR) Serial.println("------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));
    if(DEBUG_SENSOR) Serial.println("");
  }
}

void Sensor::readObstacle() {
  //  int dFL, dFM, dFR;
  //  int dLF, dLB;
  //  int dR;
  //
  //  readSensorRawValues(&dFL, &dFM, &dFR, &dLF, &dLB, &dR);
  //
  //  *oFL = dFL < sFL[0] ? gShort[0] : dFL < sFL[1] ? gShort[1] : dFL < sFL[2] ? gShort[2] : dFL < sFL[3] ? gShort[3] : gShort[4];
  //  *oFM = dFM < sFM[0] ? gShort[0] : dFM < sFM[1] ? gShort[1] : dFM < sFM[2] ? gShort[2] : dFM < sFM[3] ? gShort[3] : gShort[4];
  //  *oFR = dFR < sFR[0] ? gShort[0] : dFR < sFR[1] ? gShort[1] : dFR < sFR[2] ? gShort[2] : dFR < sFR[3] ? gShort[3] : gShort[4];
  //
  //  *oLF = dLF < sLF[0] ? gShort[0] : dLF < sLF[1] ? gShort[1] : dLF < sLF[2] ? gShort[2] : dLF < sLF[3] ? gShort[3] : gShort[4];
  //  *oLB = dLB < sLB[0] ? gShort[0] : dLB < sLB[1] ? gShort[1] : dLB < sLB[2] ? gShort[2] : dLB < sLB[3] ? gShort[3] : gShort[4];
  //
  //  *oR = dR < sR[0] ? gLong[0] : dR < sR[1] ? gLong[1] : dR < sR[2] ? gLong[2] : dR < sR[3] ? gLong[3] : dR < sR[4] ? gLong[4] : dR < sR[5] ? gLong[5] : gLong[6];
  readFrontLeftObstacle();
  readFrontMidObstacle();
  readFrontRightObstacle();
  readLeftFrontObstacle();
  readLeftBackObstacle();
  readRightObstacle();
}

void Sensor::readFrontLeftObstacle() {
  readFrontLeftSensorRawValues();
  oFL = rFL < sFL[0] ? gShort[0] : rFL < sFL[1] ? gShort[1] : rFL < sFL[2] ? gShort[2] : rFL < sFL[3] ? gShort[3] : gShort[4];
}

void Sensor::readFrontMidObstacle() {
  readFrontMidSensorRawValues();
  oFM = rFM < sFM[0] ? gShort[0] : rFM < sFM[1] ? gShort[1] : rFM < sFM[2] ? gShort[2] : rFM < sFM[3] ? gShort[3] : gShort[4];
}
void Sensor::readFrontRightObstacle() {
  readFrontRightSensorRawValues();
  oFR = rFR < sFR[0] ? gShort[0] : rFR < sFR[1] ? gShort[1] : rFR < sFR[2] ? gShort[2] : rFR < sFR[3] ? gShort[3] : gShort[4];
}
void Sensor::readLeftFrontObstacle() {
  readLeftFrontSensorRawValues();
  oLF = rLF < sLF[0] ? gShort[0] : rLF < sLF[1] ? gShort[1] : rLF < sLF[2] ? gShort[2] : rLF < sLF[3] ? gShort[3] : gShort[4];
}
void Sensor::readLeftBackObstacle() {
  readLeftBackSensorRawValues();
  oLB = rLB < sLB[0] ? gShort[0] : rLB < sLB[1] ? gShort[1] : rLB < sLB[2] ? gShort[2] : rLB < sLB[3] ? gShort[3] : gShort[4];
}
void Sensor::readRightObstacle() {
  readRightSensorRawValues();
  oR = rR < sR[0] ? gLong[0] : rR < sR[1] ? gLong[1] : rR < sR[2] ? gLong[2] : rR < sR[3] ? gLong[3] : rR < sR[4] ? gLong[4] : rR < sR[5] ? gLong[5] : gLong[6];
}

void Sensor::readSensorRawValues() {
  readFrontSensorRawValues();
  readLeftSensorRawValues();
  readRightSensorRawValues();
}

void Sensor::readFrontSensorRawValues() {
  readFrontLeftSensorRawValues();
  readFrontMidSensorRawValues();
  readFrontRightSensorRawValues();
}

void Sensor::readLeftSensorRawValues() {
  readLeftFrontSensorRawValues();
  readLeftBackSensorRawValues();
}

void Sensor::readRightSensorRawValues() {
  rR = irDistance(4);
}

void Sensor::readFrontLeftSensorRawValues() {
  rFL = irDistance(1);
}

void Sensor::readFrontMidSensorRawValues() {
  rFM = irDistance(2);
}

void Sensor::readFrontRightSensorRawValues() {
  rFR = irDistance(3);
}

void Sensor::readLeftFrontSensorRawValues() {
  rLF = irDistance(5);
}

void Sensor::readLeftBackSensorRawValues() {
  rLB = irDistance(6);
}

bool Sensor::hasObstacleForCalib() {
  readObstacle();

  oFL = rFL < sFL[0] ? gShort[0] : rFL < sFL[1] ? gShort[1] : rFL < sFL[2] ? gShort[2] : rFL < sFL[3] ? gShort[3] : gShort[4];
  oFM = rFM < sFM[0] ? gShort[0] : rFM < sFM[1] ? gShort[1] : rFM < sFM[2] ? gShort[2] : rFM < sFM[3] ? gShort[3] : gShort[4];
  oFR = rFR < sFR[0] ? gShort[0] : rFR < sFR[1] ? gShort[1] : rFR < sFR[2] ? gShort[2] : rFR < sFR[3] ? gShort[3] : gShort[4];

  oLF = rLF < sLF[0] ? gShort[0] : rLF < sLF[1] ? gShort[1] : rLF < sLF[2] ? gShort[2] : rLF < sLF[3] ? gShort[3] : gShort[4];
  oLB = rLB < sLB[0] ? gShort[0] : rLB < sLB[1] ? gShort[1] : rLB < sLB[2] ? gShort[2] : rLB < sLB[3] ? gShort[3] : gShort[4];

  oR = rR < sR[0] ? gShort[0] : rR < sR[1] ? gShort[1] : gShort[4];

  if (DEBUG_SENSOR) Serial.println("hasObsForCal: " + String(oFL) + " | " + String(oFM) + " | " + String(oFR) + " | " + String(oLF) + " | " + String(oLB) + " | " + String(oR));

  if ((oFL <= gShort[3] || oFM <= gShort[3] || oFR <= gShort[3]) && (oLF <= gShort[3] && oLB <= gShort[3]))
    return true;

  return false;
}

int Sensor::hasObstacleForSelfCalib() {
  readSensorRawValues();

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println(String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | " );

  oLF = rLF < sLF[0] ? gShort[0] : rLF < sLF[1] ? gShort[1] : rLF < sLF[2] ? gShort[2] : rLF < sLF[3] ? gShort[3] : gShort[4];
  oLB = rLB < sLB[0] ? gShort[0] : rLB < sLB[1] ? gShort[1] : rLB < sLB[2] ? gShort[2] : rLB < sLB[3] ? gShort[3] : gShort[4];

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println("hasObsForSelfCal: " + String(oLF) + " | " + String(oLB));

  if (oLF != oLB)
    return 0;

//  if ((oLF < gShort[2] && rLF > sLF_Limit[3]) || (oLB < gShort[2] && rLB > sLB_Limit[3]) ||
//      (oLF == gShort[0] && rLF < sLF_Limit[2]) || (oLB == gShort[0] && rLB < sLB_Limit[2]))
//    return 2;
//  else 
  if (oLF < gShort[1] || oLB < gShort[1])//(*rLF < (sLF[1] * 0.75)) && (*rLB < (sLB[1] * 0.75)))
    return 1;

  return 0;
}

int Sensor::hasObstacleForSelfCalib_Front() {
  readSensorRawValues();

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println(String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | " );

  oFL = rFL < sFL[0] ? gShort[0] : rFL < sFL[1] ? gShort[1] : rFL < sFL[2] ? gShort[2] : rFL < sFL[3] ? gShort[3] : gShort[4];
  oFR = rFR < sFR[0] ? gShort[0] : rFR < sFR[1] ? gShort[1] : rFR < sFR[2] ? gShort[2] : rFR < sFR[3] ? gShort[3] : gShort[4];
  oFM = rFM < sFM[0] ? gShort[0] : rFM < sFM[1] ? gShort[1] : rFM < sFM[2] ? gShort[2] : rFM < sFM[3] ? gShort[3] : gShort[4];

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println("hasObstacleForSelfCalib_Front: " + String(oFL) + " | " + String(oFM) + " | " + String(oFR));

  if(oFM > gShort[0] || rFM < sFM_Limit[0])
    return 0;

  if (oFM == gShort[0] && oFR == gShort[0])
    return 2; //Front Right and Mid
  else if (oFM == gShort[0] && oFL == gShort[0])
    return 1; //Front Left and Mid
    
  return 0;
}

int Sensor::hasObstacleForLeftCalib() {
  readSensorRawValues();

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println(String(rFL) + " | " + String(rFM) + " | " + String(rFR) + " | " + String(rLF) + " | " + String(rLB) + " | " + String(rR) + " | " );

  oLF = rLF < sLF[0] ? gShort[0] : rLF < sLF[1] ? gShort[1] : rLF < sLF[2] ? gShort[2] : rLF < sLF[3] ? gShort[3] : gShort[4];
  oLB = rLB < sLB[0] ? gShort[0] : rLB < sLB[1] ? gShort[1] : rLB < sLB[2] ? gShort[2] : rLB < sLB[3] ? gShort[3] : gShort[4];

  if (DEBUG_SENSOR || DEBUG_CALIB) Serial.println("hasObsForSelfCal: " + String(oLF) + " | " + String(oLB));

  if (oLF != oLB)
    return 0;

  if ((oLF < gShort[2] && rLF > sLF_Limit[3]) || (oLB < gShort[2] && rLB > sLB_Limit[3]) ||
      (oLF == gShort[0] && rLF < sLF_Limit[2]) || (oLB == gShort[0] && rLB < sLB_Limit[2]))
    return 2;

  return 0;
}

//@Depreciated
//void Sensor::readObstacle(int *oF, int *oL, int *oR) {
//
//  //NEED TO READ THEM ONE BY ONE, ONLY WHEN NEEDED
//  //Detect front sensors
//  if (*oF == 0) {
//    int dFL, dFM = irDistance(2), dFR;
//    if (dFM < sFM[0]) {
//      *oF = 0;
//      if (DEBUG_SENSOR) Serial.println("F1");
//    } else if (dFM > sFM[0]) {
//      dFL = irDistance(1);
//      if (DEBUG_SENSOR) Serial.println("F2");
//      if (dFL < sFL[0]) {
//        *oF = 0;
//        if (DEBUG_SENSOR) Serial.println("F3");
//      } else {
//        dFR = irDistance(3);
//        if (DEBUG_SENSOR) Serial.println("F4");
//
//        if (dFR < sFR[0]) {
//          *oF = 0;
//          if (DEBUG_SENSOR) Serial.println("F5");
//        }
//
//        if (dFM < sFM[1] || dFL < sFL[1] || dFR < sFR[1]) {
//          *oF = 1;
//          if (DEBUG_SENSOR) Serial.println("F6");
//        } else if (dFM < sFM[2] || dFL < sFL[2] || dFR < sFR[2]) {
//          *oF = 2;
//          if (DEBUG_SENSOR) Serial.println("F7");
//        } else {
//          *oF = 3;
//          if (DEBUG_SENSOR) Serial.println("F8");
//        }
//      }
//    }
//  }
//
//  //Detect left sensors
//  if (*oL == 0) {
//    int dLF = irDistance(5), dLB;
//    if (dLF < sLF[0]) {
//      *oL = 0;
//      if (DEBUG_SENSOR) Serial.println("L1");
//    } else if (dLF > sLF[0]) {
//      dLB = irDistance(6);
//      if (DEBUG_SENSOR) Serial.println("L2");
//      if (dLB < sLB[0]) {
//        *oL = 0;
//        if (DEBUG_SENSOR) Serial.println("L3");
//      }
//
//      if (dLF < sLF[1] || dLB < sLB[1]) {
//        *oL = 1;
//        if (DEBUG_SENSOR) Serial.println("L4");
//      } else if (dLF < sLF[2] || dLB < sLB[2]) {
//        *oL = 2;
//        if (DEBUG_SENSOR) Serial.println("L5");
//      } else {
//        *oL = 3;
//        if (DEBUG_SENSOR) Serial.println("L6");
//      }
//    }
//  }
//
//  if (*oR == 0) {
//    int dR = irDistance(4);
//    if (dR < sR[0]) {
//      *oR = 0;
//      if (DEBUG_SENSOR) Serial.println("R1");
//    } else if (dR < sR[1]) {
//      *oR = 1;
//      if (DEBUG_SENSOR) Serial.println("R2");
//    } else if (dR < sR[2]) {
//      *oR = 2;
//      if (DEBUG_SENSOR) Serial.println("R3");
//    } else if (dR < sR[3]) {
//      *oR = 3;
//      if (DEBUG_SENSOR) Serial.println("R4");
//    } else if (dR < sR[4]) {
//      *oR = 4;
//      if (DEBUG_SENSOR) Serial.println("R5");
//    } else {
//      *oR = 5;
//      if (DEBUG_SENSOR) Serial.println("R6");
//    }
//  }
//}

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

static int pFL = 0, pFM = 0, pFR = 0, pLF = 0, pLB = 0, pR = 0, i = 0;

void Sensor::printAllSensors() {
  //  int oFL, oFM, oFR, oLF, oLB, oR;
  //  readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
  //
  //  Serial.println("a" + String(oLF) + "," + String(oLB) + "," + String(oFL) + "," + String(oFR) + "," + String(oFM) + "," + String(oR));

  readObstacle();
  int i = 0;
  
  if (SENSOR_STABILITY_TEST) {
    aFL[0] = oFL; aFM[0] = oFM; aFR[0] = oFR; aLF[0] = oLF; aLB[0] = oLB; aR[0] = oR;
    readObstacle();
    aFL[1] = oFL; aFM[1] = oFM; aFR[1] = oFR; aLF[1] = oLF; aLB[1] = oLB; aR[1] = oR;
    readObstacle();
    aFL[2] = oFL; aFM[2] = oFM; aFR[2] = oFR; aLF[2] = oLF; aLB[2] = oLB; aR[2] = oR;

    mergeSort(aFL, 0, 2);
    mergeSort(aFM, 0, 2);
    mergeSort(aFR, 0, 2);
    mergeSort(aLF, 0, 2);
    mergeSort(aLB, 0, 2);
    mergeSort(aR, 0, 2);

    bool sensorCorrection;

    do {
      sensorCorrection = false;
      if (DEBUG_SENSOR) Serial.println("\nSensor Reading Loop: " + String(i));

      if (aFL[2] != aFL[0] && aFL[0] != aFL[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh FL | " + String(aFL[2]) + " - " + String(aFL[0]));
        readFrontLeftObstacle();
        if(aFL[0] == aFL[1])
          aFL[2] = oFL;
        else 
          aFL[0] = oFL;
        sensorCorrection = true;
      }
      if (aFM[2] != aFM[0] && aFM[0] != aFM[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh FM | " + String(aFM[2]) + " - " + String(aFM[0]));
        readFrontMidObstacle();
        if(aFM[0] == aFM[1])
          aFM[2] = oFM;
        else 
          aFM[0] = oFM;
        sensorCorrection = true;
      }
      if (aFR[2] != aFR[0] && aFR[0] != aFR[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh FR | " + String(aFR[2]) + " - " + String(aFR[0]));
        readFrontRightObstacle();
        if(aFR[0] == aFR[1])
          aFR[2] = oFR;
        else 
          aFR[0] = oFR;
        sensorCorrection = true;
      }
      if (aLF[2] != aLF[0] && aLF[0] != aLF[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh LF | " + String(aLF[2]) + " - " + String(aLF[0]));
        readLeftFrontObstacle();
        if(aLF[0] == aLF[1])
          aLF[2] = oLF;
        else 
          aLF[0] = oLF;
        sensorCorrection = true;
      }
      if (aLB[2] != aLB[0] && aLB[0] != aLB[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh LB | " + String(aLB[2]) + " - " + String(aLB[0]));
        readLeftBackObstacle();
        if(aFL[0] == aFL[1])
          aLB[2] = oLB;
        else 
          aLB[0] = oLB;
        sensorCorrection = true;
      }
      if (aR[2] != aR[0] && aR[0] != aR[1]) {
        if (DEBUG_SENSOR) Serial.println("\nRefresh R | " + String(aR[2]) + " - " + String(aR[0]));
        readRightObstacle();
        if(aFL[0] == aFL[1])
          aR[2] = oR;
        else 
          aR[0] = oR;
        sensorCorrection = true;
      }

      mergeSort(aFL, 0, 2);
      mergeSort(aFM, 0, 2);
      mergeSort(aFR, 0, 2);
      mergeSort(aLF, 0, 2);
      mergeSort(aLB, 0, 2);
      mergeSort(aR, 0, 2);
    } while (sensorCorrection && (i++ < 7));

    if (DEBUG_SENSOR) Serial.println("\na" + String(aLF[0]) + "," + String(aLB[0]) + "," + String(aFL[0]) + "," + String(aFR[0]) + "," + String(aFM[0]) + "," + String(aR[0]));
    if (i > 5) {
      if (pFL == aFL[1] && pLF == aLF[1] && pR == aR[1] ) {
        if(DEBUG_SENSOR) Serial.println("a0,0,0,0,0,0" + String(i));
      }
      i = 0;
    }
    i++;
    pFL = aFL[1]; pFM = aFM[1]; pFR = aFR[1]; pLF = aLF[1]; pLB = aLB[1]; pR = aR[1];
    Serial.println("a" + String(aLF[1]) + "," + String(aLB[1]) + "," + String(aFL[1]) + "," + String(aFR[1]) + "," + String(aFM[1]) + "," + String(aR[1]));
    if (DEBUG_SENSOR) Serial.println("a" + String(aLF[2]) + "," + String(aLB[2]) + "," + String(aFL[2]) + "," + String(aFR[2]) + "," + String(aFM[2]) + "," + String(aR[2]));
  } else
    Serial.println("a" + String(oLF) + "," + String(oLB) + "," + String(oFL) + "," + String(oFR) + "," + String(oFM) + "," + String(oR));
}

void Sensor::printAllSensorsRAW() {
  bFL[3], bFM[3], bFR[3], bLF[3], bLB[3], bR[3];

  readSensorRawValues();
  if (SENSOR_STABILITY_TEST) {
    bFL[0] = rFL; bFM[0] = rFM; bFR[0] = rFR; bLF[0] = rLF; bLB[0] = rLB; bR[0] = rR;
    readSensorRawValues();
    bFL[1] = rFL; bFM[1] = rFM; bFR[1] = rFR; bLF[1] = rLF; bLB[1] = rLB; bR[1] = rR;
    readSensorRawValues();
    bFL[2] = rFL; bFM[2] = rFM; bFR[2] = rFR; bLF[2] = rLF; bLB[2] = rLB; bR[2] = rR;

    mergeSort(bFL, 0, 2);
    mergeSort(bFM, 0, 2);
    mergeSort(bFR, 0, 2);
    mergeSort(bLF, 0, 2);
    mergeSort(bLB, 0, 2);
    mergeSort(bR, 0, 2);

    if (bFL[2] - bFL[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh FL | " + String(bFL[2]) + " - " + String(bFL[0]));
      readFrontLeftSensorRawValues();
      bFL[2] = rFL;
    }
    if (bFM[2] - bFM[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh FM | " + String(bFM[2]) + " - " + String(bFM[0]));
      readFrontMidSensorRawValues();
      bFM[2] = rFM;
    }
    if (bFR[2] - bFR[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh FR | " + String(bFR[2]) + " - " + String(bFR[0]));
      readFrontRightSensorRawValues();
      bFR[2] = rFR;
    }
    if (bLF[2] - bLF[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh LF | " + String(bLF[2]) + " - " + String(bLF[0]));
      readLeftFrontSensorRawValues();
      bLF[2] = rLF;
    }
    if (bLB[2] - bLB[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh LB | " + String(bLB[2]) + " - " + String(bLB[0]));
      readLeftBackSensorRawValues();
      bLB[2] = rLB;
    }
    if (bR[2] - bR[0] > 100) {
      if (DEBUG_SENSOR) Serial.println("\nRefresh R | " + String(bR[2]) + " - " + String(bR[0]));
      readRightSensorRawValues();
      bR[2] = rR;
    }

    if (DEBUG_SENSOR) Serial.println("\na" + String(bLF[0]) + "," + String(bLB[0]) + "," + String(bFL[0]) + "," + String(bFR[0]) + "," + String(bFM[0]) + "," + String(bR[0]));
    Serial.println("a" + String(bLF[1]) + "," + String(bLB[1]) + "," + String(bFL[1]) + "," + String(bFR[1]) + "," + String(bFM[1]) + "," + String(bR[1]));
    if (DEBUG_SENSOR) Serial.println("a" + String(bLF[2]) + "," + String(bLB[2]) + "," + String(bFL[2]) + "," + String(bFR[2]) + "," + String(bFM[2]) + "," + String(bR[2]));
  } else
    Serial.println("a" + String(rLF) + "," + String(rLB) + "," + String(rFL) + "," + String(rFR) + "," + String(rFM) + "," + String(rR));
}
