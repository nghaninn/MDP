#include "Movement.h"
#include "Global.h"

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
  if (DEBUG_MOVEMENT) Serial.println("moveFrontShort()" + String(distance));

  double pid;
  this->integral = 0;
  PrevTicks = 0;
  motor->resetTicks();

  int M1setSpeed, M2setSpeed;

  for (int i = 0; i < MSpeed; i += 50) {
    pid = computePID();
    M1setSpeed = -(i - pid);
    M2setSpeed = -(i + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  while (motor->getM1Ticks() < max(distance * 0.9, distance - 50)) {
    pid = computePID();
    M1setSpeed = -(MSpeed + offset - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }
  
  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));

  int i;
  for (i = 100; (motor->getM1Ticks() < distance && i < MSpeed); i += 50) {
    pid = computePID();
    M1setSpeed = -(MSpeed - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }
  
  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));

  while (motor->getM1Ticks() < distance) {
    pid = computePID();
    M1setSpeed = -(MSpeed - pid - i + 50);
    M2setSpeed = -(MSpeed + pid - i + 50);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  brake();
  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
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
//  delay(100);
}

double Movement::computePID() {
  double kp, ki, kd, p, i, d, pid, error;

// 250: p:8.4 i:0.49 d:0.33

  kp = 0;//8.4;//7.9;
  ki = 0;//0.49;//0.45;
  kd = 0;//0.33;//0.38;

//  kp = 7.9;
//  ki = 0.46;
//  kd = 0.38;

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
  else if (forward == 4)
    moveFrontShort(d40);
  else if (forward == 5)
    moveFrontShort(d50);
  else if (forward == 6)
    moveFront(d60);
  else if (forward == 7)
    moveFront(d70);
  else if (forward == 8)
    moveFront(d80);
  else if (forward == 9)
    moveFront(d90);
  else if (forward == 10)
    moveFront(d100);
  else if (forward == 11)
    moveFront(d110);
  else if (forward == 12)
    moveFront(d120);
  else if (forward == 13)
    moveFront(d130);
  else if (forward == 14)
    moveFront(d140);
  else if (forward == 15)
    moveFront(d150);
  else if (forward == 16)
    moveFront(d160);
  else if (forward == 17)
    moveFront(d170);
//  else if (forward == 18)
//    moveFront(d180);
  else
    moveFront10CM();
}

void Movement::moveFront5CM() {
  moveFrontShort(Distance_5CM);
}

void Movement::moveFront10CM() {
  moveFrontShort(d10);
}

void Movement::moveFront20CM() {
  moveFrontShort(d20);
}

void Movement::moveFront30CM() {
  moveFrontShort(d30);
}

void Movement::moveFrontShort(int distance) {
  if (DEBUG_MOVEMENT) Serial.println("moveFrontShort()" + String(distance));

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
    //    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  while (motor->getM1Ticks() < max(distance * 0.9, distance - 50)) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed + offset - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));

  int i;
  for (i = 100; (motor->getM1Ticks() < distance && i < MSpeed); i += 50) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  while (motor->getM1Ticks() < distance) {
    pid = computePIDShort();
    M1setSpeed = -(MSpeed - pid - i + 50);
    M2setSpeed = -(MSpeed + pid - i + 50);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    //    if (DEBUG_MOVEMENT) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //    if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
    //    if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
    delay(delayTime);
  }

  brake();
  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
//  delay(100);
}


double Movement::computePIDShort() {
  double kp, ki, kd, p, i, d, pid, error;

  kp = 8.4;//7.9;
  ki = 0.49;//0.45;
  kd = 0.33;//0.38;

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
//    delay (100);
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
      //      if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      //      if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
      delay(delayTime);
    }
  } else {
    while (motor->getM1Ticks() < distance + L_offset) {
      pid = computePID_right(degree);
      int M1setSpeed = (MSpeedR);
      int M2setSpeed = -(MSpeedR + pid);
      md.setSpeeds (M1setSpeed, M2setSpeed);
      //      if (DEBUG_MOVEMENT) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      //      if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
      delay(delayTime);
    }
  }
  brake();
  if (DEBUG_MOVEMENT) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(motor->getM1Ticks()) + ") - M2Ticks(" + String(motor->getM2Ticks()) + ") = " + String(motor->getM1Ticks() - motor->getM2Ticks()));
  if (DEBUG_MOVEMENT) Serial.println("Total Error: " + String(integral));
//  delay(100);
}

int Movement::computePID_right(int angle) {
  double kp, ki, kd, p, i, d, pid, error;

  kp = 18;
  ki = 0.63;
  kd = 0;

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

void Movement::moveSmall(int distance) {

  motor->resetTicks();

  while (motor->getM1Ticks() < abs(distance)) {
    int M1setSpeed = -(MSpeedC) * (distance > 0 ? 1 : -1);
    int M2setSpeed = -(MSpeedC) * (distance > 0 ? 1 : -1);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    delay(10);
  }

  brake();
}

void Movement::rotateSmall(int distance) { //rotate right by default

  motor->resetTicks();

  while (motor->getM1Ticks() < abs(distance)) {
    int M1setSpeed = -(MSpeedC) * (distance > 0 ? 1 : -1);
    int M2setSpeed = (MSpeedC) * (distance > 0 ? 1 : -1);
    md.setSpeeds (M1setSpeed, M2setSpeed);
    delay(10);
  }

  brake();
}
