#include "Movement.h"

Movement::Movement() {
  motor = new Motor();
}

void Movement::newBatt() {
  moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180);
}

void Movement::brake() {
  md.setBrakes(400, 400);
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    MOVING FORWARD AND REVERSE
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void Movement::moveFront(double distance) {
  double pid;
  this->integral = 0;
  PrevTicks = 0;
  motor->resetTicks();

  int M1setSpeed, M2setSpeed;

  for (int i = 0; i < MSpeed; i += 50) {
    pid = computePID();
    M1setSpeed = -(i + offset - pid);
    M2setSpeed = -(i + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    delay(10);
  }

  while (motor->getM1Ticks() < distance) {
    pid = computePID();
    M1setSpeed = -(MSpeed + offset - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  }

  for (int i = 50; (abs(M1setSpeed) > 100); i += 50) {
    pid = computePID();
    M1setSpeed = -(MSpeed + offset - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    delay(10);
  }

  brake();
  delay(100);
}

void Movement::moveReverse(double distance) {
  double pid;
  this->integral = 0;
  PrevTicks = 0;
  motor->resetTicks();

  while (motor->getM1Ticks() < distance) {
    pid = computePID();
    int M1setSpeed = (MSpeed - pid);
    int M2setSpeed = (MSpeed + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  }
  brake();
  delay(100);
}

double Movement::computePID() {
  double kp, ki, kd, p, i, d, pid, error;

  //Battery A:
  //6.03V p = 2.4 / i = 0.1 / d = 0.2
  //6.13V p = 1.8 / i = 0.1 / d = 0.1
  //6.34V p 1.9 / i = 0.1 / d = 0.1
  //Battery B:
  //6.27V+ p = 1.8 / i = 0.1 / d = 0.1
  //6.24V p = 1.7 / i = 0.1 / d = 0.3
  //6.26V p = 1.73
  //6.2V  p = 1.6 / i = 0.1 / d = 0.2

  kp = 1.8;//2.4//2.1;//2.2 (FOR 300);//1.35;// (SLIGHTLY RIGHT)//1.4 (TOO LOW STILL LEFT)//1.5 (WORKING FOR 150m) //2228;
  ki = 0.15;//0//0//.11;//0.8;
  kd = 0.15;//0.2//0//0.1;

  error = motor->getM1Ticks() - motor->getM2Ticks();
  this->integral += error;
  p = kp * error;
  i = ki * this->integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;

  return pid;
}

/* ================================================================================================================================================================
   END OF FORWARD AND REVERSE
   ================================================================================================================================================================
*/


/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    MOVING FORWARD SHORT
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void Movement::move(int forward) {
  if (forward == 1)
    moveFront10CM();
  else if (forward == 2)
    moveFront20CM();
  else if (forward == 3)
    moveFront30CM();
  else
    moveFront10CM();
}

void Movement::moveFront5CM() {
  moveFrontShort(Distance_5CM);
}

void Movement::moveFront10CM() {
  moveFrontShort(Distance_10CM);
}

void Movement::moveFront20CM() {
  moveFrontShort(Distance_20CM);
}

void Movement::moveFront30CM() {
  moveFrontShort(Distance_30CM);
}

void Movement::moveFrontShort(int distance) {
  if(DEBUG_MOVEMENT) Serial.println("moveFrontShort()" + String(distance));
  
  double pid;
  this->integral = 0;
  PrevTicks = 0;
  motor->resetTicks();

  int M1setSpeed, M2setSpeed;

  for (int i = 0; i < MSpeed; i += 50) {
    pid = computePIDShort();
    M1setSpeed = -(i - pid);
    M2setSpeed = -(i + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(10);
  }

  while (motor->getM1Ticks() < max(distance * 0.9, distance-50)) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed + offset - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
  }

  int i;
  for (i = 100; (motor->getM1Ticks() < distance && i < MSpeed); i += 50) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    //delay(5);
  }

  while (motor->getM1Ticks() < distance) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed - pid - i + 50);
    M2setSpeed = -(MSpeed + pid - i + 50);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
  }

  brake();
  delay(100);
}


double Movement::computePIDShort() {
  double kp, ki, kd, p, i, d, pid, error;

  //6.03V p = 2.4 / i = 0.1 / d = 0.2

  kp = 17;//2.4//2.1;//2.2 (FOR 300);//1.35;// (SLIGHTLY RIGHT)//1.4 (TOO LOW STILL LEFT)//1.5 (WORKING FOR 150m) //2228;
  ki = 0.6;//0.1;//0//0//.11;//0.8;
  kd = 0.8;//.4;//0.2;//0.2//0//0.1;

  error = motor->getM1Ticks() - motor->getM2Ticks();
  this->integral += error;
  p = kp * error;
  i = ki * this->integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;

  return pid;
}

/* ================================================================================================================================================================
   END OF FORWARD SHORT
   ================================================================================================================================================================
*/

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
    ROTATE
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void Movement::rotate(int degree) {
  rotateR(degree);
}

void Movement::rotateR(int degree) {
  rotate(degree, true);
}

void Movement::rotateL(int degree) {
  rotate(degree, false);
}


void Movement::rotate(int degree, boolean isRight) {
  while (degree > 0) {
    if (degree - 45 == 0 || (degree - 45) % 360 == 0) {
      rotate(45, Rotate_45deg, isRight);
      degree -= 45;
      continue;
    }
    if (degree <= 90 || (degree - 90) % 360 == 0) {
      rotate(90, Rotate_90deg, isRight);
      degree -= 90;
    } else if (degree <= 180 || (degree - 180) % 360 == 0) {
      rotate(180, Rotate_180deg, isRight);
      degree -= 180;
    } else if (degree <= 270) {
      rotate(270, Rotate_270deg, isRight);
      degree -= 270;
    } else if (degree <= 360) {
      rotate(360, Rotate_360deg, isRight);
      degree -= 360;
    }
    delay (100);
  }
}

void Movement::rotate(int degree, double distance, boolean isRight) {
  double pid;
  this->integral = 0;
  PrevTicks = 0;
  motor->resetTicks();

  if (isRight) {
    while (motor->getM1Ticks() < distance) {
      pid = computePID_right(degree);
      int M1setSpeed = -(MSpeedR);
      int M2setSpeed = (MSpeedR + pid);
      md.setSpeeds (M1setSpeed, M2setSpeed);
      if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    }
  } else {
    while (motor->getM1Ticks() < distance + L_offset) {
      pid = computePID_right(degree);
      int M1setSpeed = (MSpeedR);
      int M2setSpeed = -(MSpeedR + pid);
      md.setSpeeds (M1setSpeed, M2setSpeed);
      if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    }
  }
  brake();
  delay(100);
}

int Movement::computePID_right(int angle) {

  //  if(DEBUG_MOVEMENT) Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
  double kp, ki, kd, p, i, d, pid, error;

  //Battery A:
  //6.03V p = 2.4
  //6.13V p = 1.8
  //6.3V p = 2.87 / i = 1
  //Battery B:
  //6.27V+ p = 1.8
  //6.24V p = 1.7
  //6.26V p = 1.73

  //After removing serial print
  //Battery A
  // 6.18V p = 10; i = 0.1; 12 (unstable state)
  //Battery B
  // 6.23V p = 8; 10 (unstable state)

  kp = 10;// (FOR 360);//1.35;// (SLIGHTLY RIGHT)//1.4 (TOO LOW STILL LEFT)//1.5 (WORKING FOR 150m) //2228;
  ki = 0.1;//.11;//0.8;
  kd = 0;//0.1;

  error = motor->getM1Ticks() - motor->getM2Ticks();
  this->integral += error;
  p = kp * error;
  i = ki * this->integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;

  return pid;
}

/* ================================================================================================================================================================
   END OF ROTATE
   ================================================================================================================================================================
*/
