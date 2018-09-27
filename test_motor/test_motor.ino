#include <EnableInterrupt.h>
#include <DualVNH5019MotorShield.h>
#include "math.h"

#define ir1 A0 //FL
#define ir2 A1 //FM
#define ir3 A2 //FR
#define ir4 A3 //RF
#define ir5 A4 //LF
#define ir6 A5 //LB

#define model 1080

//motors pin
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;


volatile long M1Ticks, M2Ticks;
long PrevTicks;
const int MSpeed = 250;
const int MSpeedR = 150;
const int offset = -18; //M1 Speed offset (Going abt 20pwm faster)
bool activate = true;
char commands;

const int Distance_10CM = 245;//278;
const int Distance_20CM = Distance_10CM * 2;
const int Distance_30CM = Distance_10CM * 3.23;
const int d180 = 4750;//5004;//2500;//
const int d180_stop = 5004;

const int Rotate_45deg = 185;
const int Rotate_90deg = 385;//357;
const int Rotate_180deg = Rotate_90deg * 2.057;
const int Rotate_270deg = Rotate_90deg * 3.3;
const int Rotate_360deg = Rotate_90deg * 4.51;


////For paper testing
//const int Rotate_45deg = 178;
//const int Rotate_90deg = 393; //374
//const int Rotate_180deg = Rotate_90deg * 2.04;//2.1105;
//const int Rotate_270deg = Rotate_90deg * 3.083;
//const int Rotate_360deg = Rotate_90deg * 4.113;
const int Rotate_720deg = 3243;
const int Rotate_900deg = 4070;
const int Rotate_1080deg = 4870;

const boolean DEBUG_MOTO = true;
const boolean DEBUG_MOVE = true;
const boolean DEBUG_SENSOR = true;

double integral;

DualVNH5019MotorShield md;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//(19200);
  if(DEBUG_MOTO) Serial.println("Motor Test");
  enableInterrupt(M1A, encoder1, RISING);
  enableInterrupt(M2A, encoder2, RISING);
  md.init();
  
  pinMode (ir1, INPUT);
  pinMode (ir2, INPUT);
  pinMode (ir3, INPUT);
  pinMode (ir4, INPUT);
  pinMode (ir5, INPUT);
  pinMode (ir6, INPUT);
  analogReference(DEFAULT);
}

boolean newBat = false;

int testing = 1;

void loop() {
  // put your main code here, to run repeatedly:
  
  if(newBat) {
    moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180); moveFront(d180); moveReverse(d180);
  }
  
  if(testing == 1)
    move(1); 
  else if (testing == 2)
    move(2); //delay(100);
  else if (testing == 3)
    move(3);
  else if (testing == 18)
    moveFront(d180);
  else if(testing == -1)
    detectAndMove(false);
  else if (testing == 90)
    rotate(90);
  else if (testing == 180)
    rotate(180);
  else if (testing == 270)
    rotate(270);
  else if (testing == 360)
    rotate(360);
  else if(testing == 5) {
    rotate(90); delay(1000);
    rotate(90); delay(1000);
    rotate(90); delay(1000);
    rotate(90); delay(1000);
  } else if (testing >= 720) {
    rotate(testing);
  } else if (testing == 0) {
    detectAll();
  }
    
  delay(100);
  while(activate){
    while (Serial.available() > 0){
      commands = Serial.read();
      delay(100);
      switch(commands){
      case 'w': moveFront10CM(); break;
      case 's': moveReverse(Distance_10CM); break;
      case 'r': rotate(90); break;
      case 'f': activate = false; break;
      default: break;
      }
    }
  }
  while(!activate){
    md.setBrakes(0, 0);
  }
}

void encoder1(){
  M1Ticks++;
}

void encoder2(){
  M2Ticks++;
}

void brake(){
  md.setBrakes(400, 400);
}

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  MOVING FORWARD AND REVERSE
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

void moveFront(double distance){
  double pid;
  integral = 0;
  PrevTicks = 0;
  M1Ticks = 0;
  M2Ticks = 0;
  
  int M1setSpeed, M2setSpeed;
  
  for(int i = 0; i < MSpeed; i+= 50) {
    pid = computePID();
    M1setSpeed = -(i + offset - pid);
    M2setSpeed = -(i + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    delay(10);
  }

  while (M1Ticks < distance) {
    pid = computePID();
    M1setSpeed = -(MSpeed + offset - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
  }
  if(DEBUG_MOTO) Serial.println(M1Ticks);
  
  for(int i=50; (abs(M1setSpeed) > 100); i+=50) {
    pid = computePID();
    M1setSpeed = -(MSpeed + offset - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    delay(10);
  }
  
  brake();
  delay(100);
}

void moveReverse(double distance){
  double pid;
  integral = 0;
  PrevTicks = 0;
  M1Ticks = 0;
  M2Ticks = 0;

  while (M1Ticks < distance) {
    pid = computePID();
    int M1setSpeed = (MSpeed - pid);
    int M2setSpeed = (MSpeed + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
  }
  brake();
  delay(100);
}

double computePID() {
  if(DEBUG_MOTO) Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
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
  ki = 0.1;//0//0//.11;//0.8;
  kd = 0.1;//0.2//0//0.1;

  error = M1Ticks - M2Ticks;
  integral += error;
  p = kp * error;
  i = ki*integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;
  if(DEBUG_MOTO) Serial.print("PID: ");
  if(DEBUG_MOTO) Serial.print(pid);
  if(DEBUG_MOTO) Serial.print(" | ");
  if(DEBUG_MOTO) Serial.print("Total Error: ");
  if(DEBUG_MOTO) Serial.println(integral);  
  
  return pid;
}

/* ================================================================================================================================================================
 * END OF FORWARD AND REVERSE
 * ================================================================================================================================================================
 */


/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  MOVING FORWARD SHORT
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */
 
void move(int forward) {
  if(forward == 1)
    moveFront10CM();
  else if (forward == 2)
    moveFront20CM();
  else if (forward == 3)
    moveFront30CM();
  else
    moveFront10CM();
}
void moveFront10CM() {
  moveFrontShort(Distance_10CM);
}

void moveFront20CM() {
  moveFrontShort(Distance_20CM);
}

void moveFront30CM() {
  moveFrontShort(Distance_30CM);
}

void moveFrontShort(int distance){
  double pid;
  integral = 0;
  PrevTicks = 0;
  M1Ticks = 0;
  M2Ticks = 0;
  
  int M1setSpeed, M2setSpeed;
  
  for(int i = 0; i < MSpeed; i+= 50) {
    pid = computePID();
    M1setSpeed = -(i - pid);
    M2setSpeed = -(i + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    delay(10);
  }

  while (M1Ticks < distance*0.9) {
    pid = computePID();
    M1setSpeed = -(MSpeed - pid);
    M2setSpeed = -(MSpeed + pid);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
  }
  
  for(int i=100; (M1Ticks < distance); i+=100) {
    pid = computePID();
    M1setSpeed = -(MSpeed - pid - i);
    M2setSpeed = -(MSpeed + pid - i);
    md.setSpeeds (M1setSpeed,M2setSpeed);
    if(DEBUG_MOTO) Serial.println("Decrement - M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
    //delay(5);
  }
  
  brake();
  delay(100);
}

/*
double computePID10CM() {
  if(DEBUG_MOTO) Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
  double kp, ki, kd, p, i, d, pid, error, integral;
  
  //6.03V p = 2.4 / i = 0.1 / d = 0.2

  kp = 2.4;//2.4//2.1;//2.2 (FOR 300);//1.35;// (SLIGHTLY RIGHT)//1.4 (TOO LOW STILL LEFT)//1.5 (WORKING FOR 150m) //2228;
  ki = 0.1;//0//0//.11;//0.8;
  kd = 0.2;//0.2//0//0.1;

  error = M1Ticks - M2Ticks;
  integral += error;
  p = kp * error;
  i = ki*integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;
  if(DEBUG_MOTO) Serial.print("PID: ");
  if(DEBUG_MOTO) Serial.print(pid);
  if(DEBUG_MOTO) Serial.print(" | ");
  if(DEBUG_MOTO) Serial.print("Total Error: ");
  if(DEBUG_MOTO) Serial.println(integral);  
  
  return pid;
}
*/
/* ================================================================================================================================================================
 * END OF FORWARD SHORT
 * ================================================================================================================================================================
 */

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  ROTATE
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

void rotate(int degree, boolean isRight) {
  while (degree > 0) {
    if(degree-45 == 0) {
      rotate(45, Rotate_45deg, isRight);
      degree -= 45;
      continue;
    }
    if(degree <= 90) {
      rotate(90, Rotate_90deg, isRight); 
      degree -= 90;
    } else if(degree <= 180) {
      rotate(180, Rotate_180deg, isRight);
      degree -= 180;
    } else if (degree <= 270) {
      rotate(270, Rotate_270deg, isRight);
      degree -= 270;
    } else if (degree <= 360) {
      rotate(360, Rotate_360deg, isRight);
      degree -= 360;
    } else if (degree <= 720) {
      rotate(720, Rotate_720deg, isRight);
      degree -= 720;
    } else if (degree <= 900) {
      rotate(900, Rotate_900deg, isRight);
      degree -= 900;
    } else {
      rotate(1080, Rotate_1080deg, isRight);
      degree -= 1080;
    }
    delay (100);
  }
}

void rotate(int degree) {
  rotateR(degree);
}

void rotateR(int degree) {
  rotate(degree, true);
}

void rotateL(int degree) {
  rotate(degree, false);
}

void rotate(int degree, double distance, boolean isRight) {
  double pid;
  integral = 0;
  PrevTicks = 0;
  M1Ticks = 0;
  M2Ticks = 0;

  if(isRight) {
    while (M1Ticks < distance) {
      pid = computePID_right(degree);
      int M1setSpeed = -(MSpeedR);
      int M2setSpeed = (MSpeedR + pid);
      md.setSpeeds (M1setSpeed,M2setSpeed);
      if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      if(DEBUG_MOTO) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
    }
  } else {
    while (M1Ticks < distance) {
      pid = computePID_right(degree);
      int M1setSpeed = (MSpeedR);
      int M2setSpeed = -(MSpeedR + pid);
      md.setSpeeds (M1setSpeed,M2setSpeed);
      if(DEBUG_MOTO) Serial.println("M1setSpeed: " + String(int(M1setSpeed)) + ", M2setSpeed: " + String(int(M2setSpeed)));
      if(DEBUG_MOTO) Serial.println("[PID] " + String((int)pid) + " M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
    }
  }
  brake();
  delay(100);
}

int computePID_right(int angle){
  
//  if(DEBUG_MOTO) Serial.println("[PID] M1Ticks(" + String(M1Ticks) + ") - M2Ticks(" + String(M2Ticks) + ") = " + String(M1Ticks-M2Ticks));
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

  error = M1Ticks - M2Ticks;
  integral += error;
  p = kp * error;
  i = ki*integral;
  d = kd * (error - PrevTicks);
  pid = p + i + d;
  PrevTicks = error;
//  if(DEBUG_MOTO) Serial.print("PID: ");
//  if(DEBUG_MOTO) Serial.println(pid);
  
  return pid;
}

/* ================================================================================================================================================================
 * END OF ROTATE
 * ================================================================================================================================================================
 */

/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  DETECT AND MOVE
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

//Read Obstacle
//Move forward by x grid
//Read Obstacle

//Move or turn
//Read Obstacle with front and left sensor - advance until obstacle is cleared

//

  const int forwardGrid = 15;
  int oF, oL, oR;
  int oFL, oFM, oFR, oLF, oLB;  
  
  void detectAndMove(boolean is90) {
    int forwardCount = 0;
    int horizonCount = 0;
    int faced = 0; // 0 = North, 1 = East, 2 = South, 3 = West
    int prevFaced = 0;
    boolean newCode = false;

    if (is90) {
    
      if(newCode) {
        while (forwardCount < forwardGrid) {
          
  //        readObstacle(&(oF = 0), &(oL = 0), &(oR = 0));
  //        if(DEBUG_MOVE) Serial.println("Faced: " + String(faced) + " | " + "Horizon: " + String(horizonCount) + " | " + "Forward: " + String(forwardCount) + 
  //          "------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
          readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
          if(DEBUG_MOVE) Serial.println("Faced: " + String(faced) + " | " + "Horizon: " + String(horizonCount) + " | " + "Forward: " + String(forwardCount) + 
            "\n------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));
  
          oL = min(oLF, oLB);
          oF = min(oFL, min(oFM, oFR));
  //Face 1        
          if (prevFaced == 0 && faced == 1 && oL > 0 && oF > 0) { //Block on immediate left, no sensor to see.
            move(1);
            horizonCount = horizonCount + 1;
          if(DEBUG_MOVE) Serial.println("1");
          
          } else if (faced == 1 && oLF == 0 && oF > 0) { // Block on front left
            move(3);
            horizonCount = horizonCount + 3;
          if(DEBUG_MOVE) Serial.println("2");
            
          } else if (faced == 1 && oL == 0 && oF > 0) { // Block on back left or any left got block
            move(1);
            horizonCount = horizonCount + 1;
          if(DEBUG_MOVE) Serial.println("3");
            
          } else if (faced == 1 && oL > 0 && horizonCount > 0) { // Rotate back to face front
            rotateL(90);//rotate(270); //Turn left
            faced = 0;
          if(DEBUG_MOVE) Serial.println("4");
  //Prev 1 Face 0        
          } else if (prevFaced == 1 && faced == 0 && oF > 0) { // Move forward, just cleared from obstacle
            move(3);
            forwardCount+=3;
          if(DEBUG_MOVE) Serial.println("5");
  //Face 0        
          } else if (faced == 0 && oF > 0 && oL == 0) { // Move forward, obstacle back left 
            move(1);
            forwardCount+=1;
          if(DEBUG_MOVE) Serial.println("6");
          
          } else if (faced == 0 && horizonCount > 0 && oL > 0) { // Rotate to face left
            rotateL(90);//rotate(270); //Turn left
            faced = 3;
          if(DEBUG_MOVE) Serial.println("7");
  // Face 3        
          } else if (prevFaced == 0 && faced == 3 && horizonCount > 0 && oF > 0) { // Rotate to face left
            if(oF < horizonCount) {
              move(oF);
              horizonCount = horizonCount - oF;
            } else {
              move(horizonCount);
              horizonCount = 0;
            }
          if(DEBUG_MOVE) Serial.println("8");
          
          } else if (faced == 3 && horizonCount > 0 && oL == 0 && oF > 0) { // Move left, back to original track
              move(1);
              horizonCount = horizonCount - 1;
          if(DEBUG_MOVE) Serial.println("9");
          
          } else if (faced == 3 && horizonCount == 0) { // Rotate right to face forward
            rotateR(90);
          if(DEBUG_MOVE) Serial.println("10");
  // Face 0          
          } else if(faced != 0 && oF > 0) {
            move(1); //Move forward to avoid obstacle
            horizonCount = faced == 1 ? (horizonCount + 1) : faced == 3 ? (horizonCount - 1) : horizonCount;
          if(DEBUG_MOVE) Serial.println("11");
          
          } else if (faced != 0 && oF == 0) {
            rotate(90);
            faced = (faced + 1)%4;
          if(DEBUG_MOVE) Serial.println("12");
          
          } else if (oF == 0) {
            rotate(90);
            faced = (faced + 1)%4;
          if(DEBUG_MOVE) Serial.println("13");
          
          } else if(faced == 0) {
            if(oF >= 3) {
              move(3);
              forwardCount+=3;
            } else if(oF > 0) {
              move(oF);
              forwardCount+=oF;
            }
          if(DEBUG_MOVE) Serial.println("14");
          }
          
          prevFaced = faced;
        }
      } else {
        while (forwardCount < forwardGrid) {
          readObstacle(&(oF = 0), &(oL = 0), &(oR = 0));
          if(DEBUG_MOVE) Serial.println("Faced: " + String(faced) + " | " + "Horizon: " + String(horizonCount) + " | " + "Forward: " + String(forwardCount) + "------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
          
          if (faced == 1 && oL > 0) {
            rotateL(90);//rotate(270); //Turn left
            faced = faced == 1 ? 0 : 3;
            move(3);
            move(1);
            forwardCount+=4;
    		if(DEBUG_MOVE) Serial.println("1 Cur Right Face: Rotate Left & Move 4");
          } else if (faced == 0 && horizonCount > 0 && oL > 0) {
            rotateL(90);//rotate(270); //Turn left
            faced = faced == 1 ? 0 : 3;
            move(2);
            horizonCount = horizonCount - 2;
    		if(DEBUG_MOVE) Serial.println("2 Cur Front Face: Rotate Left & Move 2");
          } else if(faced == 0 && oF > 0) {
            move(1);
            forwardCount+=1;
    		if(DEBUG_MOVE) Serial.println("3 Cur Front Face: Move " + String(1));
          } else if (faced == 3 && horizonCount == 0) {
            rotate(90);
            faced = 0;
            move(1);
            forwardCount+=1;
    		if(DEBUG_MOVE) Serial.println("4 Cur Right Face: Rotate Right & Move 1");
          } else if(faced != 0 && oF > 0) {
            move(1); //Move forward to avoid obstacle
            horizonCount = faced == 1 ? (horizonCount + 1) : faced == 3 ? (horizonCount - 1) : horizonCount;
    		if(DEBUG_MOVE) Serial.println("5 Cur Left/Right Face: Move 1");
          } else if (faced != 0 && oF == 0) {
            rotate(90);
            faced = 1;
            move(2);
            horizonCount = horizonCount + 2;
    		if(DEBUG_MOVE) Serial.println("6 Cur Left/Right Face: Rotate Right & Move 2");
          } else if (oF == 0) {
            rotate(90);
            faced = 1;
            move(2);
            horizonCount = horizonCount + 2;
    		if(DEBUG_MOVE) Serial.println("7 Cur ? Face: Rotate Right & Move 2");
          }
        }
      }

// 45 degree      
    } else {
      
      while (forwardCount < forwardGrid) {
        
//        readObstacle(&(oF = 0), &(oL = 0), &(oR = 0));
//        if(DEBUG_MOVE) Serial.println("Faced: " + String(faced) + " | " + "Horizon: " + String(horizonCount) + " | " + "Forward: " + String(forwardCount) + 
//          "------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
        readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
        if(DEBUG_MOVE) Serial.println("Faced: " + String(faced) + " | " + "Horizon: " + String(horizonCount) + " | " + "Forward: " + String(forwardCount) + 
          "\n------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));

        oL = min(oLF, oLB);
        oF = min(oFL, min(oFM, oFR));

        if /*(faced == 0 && (oFL == 2 || oFR == 2)) {
          rotate(45);
          move(3);
          move(3);
          move(1);
          rotateL(90);
          move(3);
          move(3);
          move(1);
          rotate(45);
          forwardCount += 8;
        } else if */(faced == 0 && oF == 1) {
          rotate(45);
          move(2);
          move(2);
          move(1);
          rotateL(90);
          move(2);
          move(2);
          move(1);
          rotate(45);
          forwardCount += 6;
        } else if (faced == 0 && (oF > 2 || oFM == 1)) {
          move(1);
          forwardCount += 1;
        }
      }
    }
  }
 
 /* ================================================================================================================================================================
 * END OF DETECT AND MOVE
 * ================================================================================================================================================================
 */
 
/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  DETECT
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

  const char dir = 'l';//'f';//'r';//
  const int NB_SAMPLE = 10;
  int ir_val[NB_SAMPLE];

  void detect() {
    int oF, oL, oR;
    while(1){
      
      readObstacle(&(oF = 0), &(oL = 0), &(oR = 0));
      if(DEBUG_MOVE) Serial.println("------------------------------F:" + String(oF) + " | L:" + String(oL) + " | R:" + String(oR));
    }
  }

  void detectAll() {
    int oFL, oFM, oFR, oLF, oFB, oR;
    int i = 0;
    while(1) {//i++ < 220){
      readObstacle(&(oFL = 0), &(oFM = 0), &(oFR = 0), &(oLF = 0), &(oLB = 0), &(oR = 0));
//      if(DEBUG_MOVE) Serial.println("------------------------------FL:" + String(oFL) + " | FM:" + String(oFM) + " | FR:" + String(oFR) + " | LF:" + String(oLF) + " | LB:" + String(oLB) + " | R:" + String(oR));
    }
  }

  const int rFL []PROGMEM = {100, 200, 300};
  const int rFM []PROGMEM = {100, 200, 300};
  const int rFR []PROGMEM = {100, 200, 300};
  const int rLF []PROGMEM = {100, 200, 300};
  const int rLB []PROGMEM = {100, 200, 300};
  const int rR  []PROGMEM = {200, 250, 330, 450, 580};
  
  void readObstacle(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR) {
    int dFL = irDistance(1), dFM = irDistance(2), dFR = irDistance(3);
    int dLF = irDistance(5), dLB = irDistance(6);
    int dR = irDistance(4);
    
    *oFL = dFL < rFL[0] ? 0 : dFL < rFL[1] ? 1 : dFL < rFL[2] ? 2 : 3;
    *oFM = dFM < rFM[0] ? 0 : dFM < rFM[1] ? 1 : dFM < rFM[2] ? 2 : 3;
    *oFR = dFR < rFR[0] ? 0 : dFR < rFR[1] ? 1 : dFR < rFR[2] ? 2 : 3;
    
    *oLF = dLF < rLF[0] ? 0 : dLF < rLF[1] ? 1 : dLF < rLF[2] ? 2 : 3;
    *oLB = dLB < rLB[0] ? 0 : dLB < rLB[1] ? 1 : dLB < rLB[2] ? 2 : 3;
    
    *oR = dR < rR[0] ? 0 : dR < rR[1] ? 1 : dR < rR[2] ? 2 : dR < rR[3] ? 3 : dR < rR[4] ? 4 : 5;
    
    if(DEBUG_SENSOR) Serial.println("");
  }
  void readObstacle(int *oF, int *oL, int *oR) {

//NEED TO READ THEM ONE BY ONE, ONLY WHEN NEEDED    
    //Detect front sensors
    if(*oF == 0) {
      int dFL, dFM = irDistance(2), dFR;
      if(dFM < rFM[0]) {
        *oF = 0; //Turn
        if(DEBUG_SENSOR) Serial.println("F1");
      } else if (dFM > rFM[0]) {
        dFL = irDistance(1);
        if(DEBUG_SENSOR) Serial.println("F2");
        if(dFL < rFL[0]) {
          *oF = 0; //Turn
          if(DEBUG_SENSOR) Serial.println("F3");
        } else {
          dFR = irDistance(3);
          if(DEBUG_SENSOR) Serial.println("F4");
          
          if(dFR < rFR[0]) {
            *oF = 0; //Turn
            if(DEBUG_SENSOR) Serial.println("F5");
          }
          
          if (dFM < rFM[1] || dFL < rFL[1] || dFR < rFR[1]) {
            *oF = 1; //Move 1 unit
            if(DEBUG_SENSOR) Serial.println("F6");
          } else if (dFM < rFM[2] || dFL < rFL[2] || dFR < rFR[2]) {
            *oF = 2; //Move 2 unit
            if(DEBUG_SENSOR) Serial.println("F7");
          } else {
            *oF = 3;
            if(DEBUG_SENSOR) Serial.println("F8");
          }
          /* else if (dFM < 43 || dFL < 50 || dFR < 30) {
            *oF = 3; //Move 3 unit
            if(DEBUG_SENSOR) Serial.println("F8");
          } else if (dFM < 54 || dFL < 69 || dFR < 40) {
            *oF = 4; //Move 3 unit
            if(DEBUG_SENSOR) Serial.println("F9");
          } else if (dFM < 70 || dFL < 68 || dFR < 50) {
            *oF = 5; //Move 3 unit
            if(DEBUG_SENSOR) Serial.println("F10");
          }*/
        }
      }
    }
    
    
//    else if (*oL == 0) {      
    if (*oL == 0) {      
      //Detect left sensors
      int dLF = irDistance(5), dLB;
      if(dLF < rLF[0]) {
        *oL = 0;
        if(DEBUG_SENSOR) Serial.println("L1");
      } else if (dLF > rLF[0]) {
        dLB = irDistance(6);
        if(DEBUG_SENSOR) Serial.println("L2");
        if(dLB < rLB[0]) {
          *oL = 0;
        if(DEBUG_SENSOR) Serial.println("L3");
        }
        
        if (dLF < rLF[1] || dLB < rLB[1]) {
          *oL = 1;
          if(DEBUG_SENSOR) Serial.println("L4");
        } else if (dLF < rLF[2] || dLB < rLB[2]) {
          *oL = 2;
          if(DEBUG_SENSOR) Serial.println("L5");
        } else {
          *oL = 3;
          if(DEBUG_SENSOR) Serial.println("L6");
        }
      } 
    }
    
//    else if (*oR == 0) {
    if (*oR == 0) {
      int dR = irDistance(4);
      if(dR < rR[0]) {
        *oR = 0;
        if(DEBUG_SENSOR) Serial.println("R1");
      } else if (dR < rR[1]) {
        *oR = 1;
        if(DEBUG_SENSOR) Serial.println("R2");
      } else if (dR < rR[2]) {
        *oR = 2;
        if(DEBUG_SENSOR) Serial.println("R3");
      } else if (dR < rR[3]) {
        *oR = 3;
        if(DEBUG_SENSOR) Serial.println("R4");
      } else if (dR < rR[4]) {
        *oR = 4;
        if(DEBUG_SENSOR) Serial.println("R5");
      } else {
        *oR = 5;
        if(DEBUG_SENSOR) Serial.println("R6");
      }
      /*else if (dR < 69) {
        *oR = 5;
        if(DEBUG_SENSOR) Serial.println("R6");
      } else if (dR < 85) {
        *oR = 6;
        if(DEBUG_SENSOR) Serial.println("R5");
      } else if (dR < 99) {
        *oR = 7;
        if(DEBUG_SENSOR) Serial.println("R5");
      }*/
    }
  }

  int irDistance(int sensor){
    for (int i=0; i<NB_SAMPLE; i++){
      ir_val[i] = analogRead(sensor == 1 ? ir1 : sensor == 2 ? ir2 : sensor == 3 ? ir3 : sensor == 4 ? ir4 : sensor == 5 ? ir5 : sensor == 6 ? ir6 : 0);
    }
  
//  ir_val = mergeSort(ir_val,0,NB_SAMPLE-1);
    mergeSort(ir_val,0,NB_SAMPLE-1);
    
    //Return in MM
    int result = sensor == 4 ? 603.74 * pow(map(ir_val[NB_SAMPLE/2], 0, 1023, 0, 5000)/1000.0, -1.16) : 277.28 * pow(map(ir_val[NB_SAMPLE/2], 0, 1023, 0, 5000)/1000.0, -1.2045);
    if(DEBUG_SENSOR) Serial.print(" | " + String(sensor) + " | " + String(ir_val[NB_SAMPLE/2]) + " | " + String(result));
    return result;
    //Return in mV
//    return ir_val[NB_SAMPLE/2];
  }
 
 /* ================================================================================================================================================================
 * END OF DETECT
 * ================================================================================================================================================================
 */
 
 
/* ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  MergeSort
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------
 */
   void merge(int arr[], int l, int m, int r) { 
    int i, j, k; 
    int n1 = m - l + 1; 
    int n2 =  r - m; 
    
    /* create temp arrays */
    int L[n1], R[n2]; 
    
    /* Copy data to temp arrays L[] and R[] */
    for (i = 0; i < n1; i++) 
    L[i] = arr[l + i]; 
    for (j = 0; j < n2; j++) 
    R[j] = arr[m + 1+ j]; 
    
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
  void mergeSort(int arr[], int l, int r) {
    if (l < r) {
      // Same as (l+r)/2, but avoids overflow for 
      // large l and h 
      int m = l+(r-l)/2; 
      
      // Sort first and second halves 
      mergeSort(arr, l, m); 
      mergeSort(arr, m+1, r); 
      
      merge(arr, l, m, r); 
    } 
  } 
 
 /* ================================================================================================================================================================
 * END OF MergeSort
 * ================================================================================================================================================================
 */
 
/*

int turnLeft_PID(int angle){
  motor1_encoder= motor2_encoder = 0;
  int correction=0,counter =0, pwm1=375, pwm2=375, target_Angle;

  if(angle<=15)
    target_Angle=angle*7.67;
  else if((angle>15)&&(angle<=30))
    target_Angle=angle*7.67;
  else if((angle>30)&&(angle<=45))
    target_Angle=angle*8.37;
  else if((angle>45)&&(angle<=90))
    target_Angle=angle*8.39;
  else if((angle>90)&&(angle<=360))
    target_Angle=angle*8.77;
  else if((angle>360)&&(angle<=720))
    target_Angle=angle*8.9;
  else if((angle>720)&&(angle<=1080))
    target_Angle=angle*8.9;
  else
    target_Angle=angle*8.9;
  
  while(1){
    if(motor2_encoder > target_Angle){  
      md.setBrakes(MAX_SPEED, MAX_SPEED);
      delay(BRAKE_DELAY);
      md.setBrakes(0, 0);
      break;
    }
  
    correction = pidControlLeft(motor1_encoder,motor2_encoder);
    md.setSpeeds(((pwm1-correction)), pwm2+correction); 
    
    Serial.print(correction);
    Serial.print(" LeftPosition: ");   Serial.print(motor1_encoder);
    Serial.print(" RightPosition: ");   Serial.print(motor2_encoder);
    Serial.print("\n");
    Serial.print(" speedPosition: ");   Serial.print((pwm1-correction));
    Serial.print(" speedtPosition: ");   Serial.print((pwm2+correction));
    Serial.print("\n");
    Serial.print(counter++);
    Serial.print("\n");
  } 
}*/


/*
int pidControl(int LeftPosition, int RightPosition){  // Used for correction during forward movement
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 1;  //0-0.1
  float Kd = 0.1;  //1-2
  float Ki = 0.8;  //0.5-1
   
  error = LeftPosition - RightPosition;
  integral =integral+ error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;
 
  return output;
}

int pidControlLeft(int LeftPosition, int RightPosition){  // Used for correction during left turn
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 1.2;  //0-0.1
  float Kd = 0;  //1-2
  float Ki = 0.1;  //0.5-1
   
  error = LeftPosition - RightPosition;
  integral =integral+ error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;
  
  return output;
}

int pidControlRight(int LeftPosition, int RightPosition){ // Used for correction during right turn
  int error,prev_error,pwm1=255,pwm2=255;
  float integral,derivative,output;
  float Kp = 1.2;  //0-0.1
  float Kd = 0;  //1-2
  float Ki = 2;  //0.5-1
   
   
  error = LeftPosition - RightPosition;
  integral =integral+ error;
  derivative = (error - prev_error);
  output = Kp*error + Ki * integral + Kd * derivative;
  prev_error = error;
 
  return output;
}*/
