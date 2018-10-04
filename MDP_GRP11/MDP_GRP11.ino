#include <EnableInterrupt.h>
#include "Calib.h"

//char commands;
String commands = "";

Movement *move;
Calib *cal;
Motor *motor;
Sensor *s;

bool DEBUG = true;
bool AUTO_SELF_CALIB = true;
bool CALIB_SENSORS_PRINTVALUES = false;

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
  delay(100);
  if (CALIB_SENSORS_PRINTVALUES)
    s->detectAll();

  while (!readCommand(&commands));

  executeCommand(commands);

  while (Serial.available()) {
    Serial.read(); //flush out all command while execution;
  }

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
  if (DEBUG) Serial.println("-Received Command: " + String(command));

  String sub_command;

  while ((sub_command = getSubString(&command, ',')).length()) {
    if (DEBUG) Serial.println("-Received Command: " + String(sub_command));
    if (sub_command.charAt(0) == 'a') {
      sub_command.remove(0, 1);
      int value = sub_command.toInt() + 1;
      Serial.println("a" + String(value));
    } else if (sub_command.charAt(0) == 'L' || sub_command.charAt(0) == 'l') {
      if (DEBUG) Serial.println("L");
      move->rotateL(90);
      if (AUTO_SELF_CALIB)
        cal->selfCalib(false);
    } else if (sub_command.charAt(0) == 'R' || sub_command.charAt(0) == 'r') {
      if (DEBUG) Serial.println("R");
      move->rotate(90);
      if (AUTO_SELF_CALIB)
        cal->selfCalib(false);
    } else if (sub_command.charAt(0) == 'C' || sub_command.charAt(0) == 'c') {
      if (DEBUG) Serial.println("C");
      if (sub_command.charAt(1) == '1')
        cal->selfCalib();
      else
        cal->calib();
    } else if (sub_command.charAt(0) == 'U' || sub_command.charAt(0) == 'u') {
      if (DEBUG) Serial.println("U");
      move->rotate(180);
      if (AUTO_SELF_CALIB)
        cal->selfCalib(false);
    } else if (sub_command.charAt(0) == 'S' || sub_command.charAt(0) == 's') {
      if (DEBUG) Serial.println("S");
      if (sub_command.charAt(1) == '1') {
        s->printAllSensorsRAW();
      } else if (sub_command.charAt(1) == '2') {
        s->detectAll();
      } else
        s->printAllSensors();
    } else if (sub_command.charAt(0) == 'M' || sub_command.charAt(0) == 'm') {
      if (DEBUG) Serial.println("M");
      if (sub_command.charAt(1) == '1')
        move->move(1);
      else if (sub_command.charAt(1) == '2')
        move->move(2);
      else if (sub_command.charAt(1) == '3')
        move->move(3);
      else
        move->move(1);
      //        move->moveSmall(-90);

      if (AUTO_SELF_CALIB)
        cal->selfCalib();
    } else if (sub_command.charAt(0) == 'z') {
      //      if (DEBUG) Serial.println("z");
      //      if (sub_command.charAt(1) == 'b')
      move->newBatt();
      //      }
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
  (*command).remove(0, i + 1);

  return temp;
}
