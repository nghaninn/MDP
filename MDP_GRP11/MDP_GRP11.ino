#include <EnableInterrupt.h>
#include "Calib.h"

bool activate = true;
//String commands;
//char commands;
String commands;

Movement *move;
Calib *cal;
Motor *motor;
Sensor *s;

bool DEBUG = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  enableInterrupt(Motor::M1A, motor1, RISING);
  enableInterrupt(Motor::M2A, motor2, RISING);

  move = new Movement();
  cal = new Calib();
  motor = new Motor();
  s = new Sensor();
}

void motor1() {
  motor->encoder1();
}

void motor2() {
  motor->encoder2();
}

void loop() {
  //  s->detectAll();
  delay(100);

  while (!readCommand(&commands));

  executeCommand(commands);

  commands = "";
}

bool readCommand(String *readVal) {
  char tmp;
  while (Serial.available())
  {
    tmp = Serial.read();
    (*readVal) += tmp;
    // append char to readVal until encounter newline
    if (tmp != '\n')
      continue;
    else
      return true; // stop blocking
  }
  // block if nothing comes in
  return false;
}

/*
   L = left turn
   R = right turn
   C = calibrate
   U = u-turn
   S = sensor around
   M1 = move 1
   M2 = move 2
   M3 = move 3

   ALGO - 1-LF 2-LB 3-FL 4-FR 5-FM 6-R - DELIMINATOR ","
   BLIND = 100
*/

bool executeCommand(String command) {
  if (DEBUG) Serial.print("-Received Command: " + String(command));
  
  String sub_command;
  
  while ((sub_command = getSubString(&command, ',')).length()) {
    if (sub_command.charAt(0) == 'a') {
      sub_command.remove(0, 1);
      int value = sub_command.toInt() + 1;
      Serial.println("b" + String(value));
    } else if (sub_command.charAt(0) == 'L' || sub_command.charAt(0) == 'l') {
      move->rotateL(90);
    } else if (sub_command.charAt(0) == 'R' || sub_command.charAt(0) == 'r') {
      move->rotate(90);
    } else if (sub_command.charAt(0) == 'C' || sub_command.charAt(0) == 'c') {
      cal->calib();
    } else if (sub_command.charAt(0) == 'U' || sub_command.charAt(0) == 'u') {
      move->rotate(180);
    } else if (sub_command.charAt(0) == 'S' || sub_command.charAt(0) == 's') {
      //return sensor values
    } else if (sub_command.charAt(0) == 'M' || sub_command.charAt(0) == 'm') {
      if(sub_command.charAt(1) == '1')
        move->move(1);
      else if(sub_command.charAt(1) == '2')
        move->move(2);
      else if(sub_command.charAt(1) == '3')
        move->move(3);
      else
        move->move(3);
    }
  }
}

String getSubString(String *command, char separator) {
  if (!(*command).length())
    return "";

  String temp = "";
  int i;

  for (i = 0; i <= (*command).length(); i++) {
    if ((*command).charAt(i) != separator) {
      temp += (*command).charAt(i);
    } else
      break;
  }
  (*command).remove(0, i+1);

  return temp;
}
