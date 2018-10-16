#include <EnableInterrupt.h>
#include "Calib.h"
#include "Global.h"

//char commands;
String commands = "";
String pre_command = "";

Movement *move;
Calib *cal;
Motor *motor;
Sensor *s;

bool AUTO_SELF_CALIB = true;
bool CALIB_SENSORS_PRINTVALUES = false;
bool ONLY_FIRST_SENSOR = false;
static int ALREADY_SENT_OUT_SENSOR = 0;

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
  if (CALIB_SENSORS_PRINTVALUES)
    s->detectAll();

  while (!readCommand(&commands));

  //  if (pre_command != commands) {
  //    while (Serial.available()) {
  //      Serial.read(); //flush out all command while execution;
  //    }
  //    pre_command = "";
  //  }

  if ((commands.charAt(0) == 'S' || commands.charAt(0) == 's')) {
    if (DEBUG) Serial.println(String(ALREADY_SENT_OUT_SENSOR));
    if (ALREADY_SENT_OUT_SENSOR > 0)
      ALREADY_SENT_OUT_SENSOR = 0;
    else if (ALREADY_SENT_OUT_SENSOR > 0)
      ALREADY_SENT_OUT_SENSOR++;
    else
      ALREADY_SENT_OUT_SENSOR = 0;
  } else
    ALREADY_SENT_OUT_SENSOR = 0;

  executeCommand(commands);

  //  while (Serial.available()) {
  //    Serial.read(); //flush out all command while execution;
  //  }

  pre_command = commands;
  commands = "";
}

bool readCommand(String *readVal) {
  char tmp;
  while (Serial.available())
  {
    tmp = Serial.read();
    (*readVal) += tmp;
    if (tmp == '\n' || tmp == '\0')
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
  if (DEBUG) Serial.println("-Received Command: " + String(command) + " | " + String(ALREADY_SENT_OUT_SENSOR));

  String sub_command;

  while ((sub_command = getSubString(&command, ',')).length()) {
    while (cal->isCalibrating);

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
      if (sub_command.charAt(1) == '1') {
        if (AUTO_SELF_CALIB)
          cal->selfCalib();
      } else if (sub_command.charAt(1) == '2')
        cal->calibFront();
      else if (sub_command.charAt(1) == '3')
        cal->calibLeft();
      else
        cal->calib();
    } else if (sub_command.charAt(0) == 'U' || sub_command.charAt(0) == 'u') {
      if (DEBUG) Serial.println("U");
      move->rotate(180);
      if (AUTO_SELF_CALIB)
        cal->selfCalib(false);
    } else if ((sub_command.charAt(0) == 'S' || sub_command.charAt(0) == 's')) {
      if (DEBUG) Serial.println("S");
      if (sub_command.charAt(1) == '1') {
        s->printAllSensorsRAW();
        ALREADY_SENT_OUT_SENSOR++;
      } else if (sub_command.charAt(1) == '2') {
        s->detectAll();
        ALREADY_SENT_OUT_SENSOR++;
      } else if (sub_command.charAt(1) == '9') {
        move->rotateL(90);
        cal->calib();
        move->rotateR(90);
        cal->selfCalib();
        s->printAllSensors();
        ALREADY_SENT_OUT_SENSOR++;
      } else if (ALREADY_SENT_OUT_SENSOR == 0) {
        s->printAllSensors();
        ALREADY_SENT_OUT_SENSOR++;
      }
    } else if (sub_command.charAt(0) == 'M' || sub_command.charAt(0) == 'm') {
      if (DEBUG) Serial.println("M");
      if (sub_command.charAt(1) == '1') {
        move->move(1);
        delay(150);
      } else if (sub_command.charAt(1) == '2') {
        move->move(2);
      } else if (sub_command.charAt(1) == '3')
        move->move(3);
      else
        move->move(1);

      if (AUTO_SELF_CALIB)
        cal->selfCalib();
    } else if (sub_command.charAt(0) == 'z') {
      //      if (DEBUG) Serial.println("z");
      //      if (sub_command.charAt(1) == 'b')
      move->newBatt();
      //      }
    }
  }
  //  delay(150);

  if (ALREADY_SENT_OUT_SENSOR == 0) {
    s->printAllSensors();
    ALREADY_SENT_OUT_SENSOR++;
  }
  if (DEBUG) Serial.println(String(ALREADY_SENT_OUT_SENSOR));
}

String getSubString(String * command, char separator) {
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
