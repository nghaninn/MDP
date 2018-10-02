#include <EnableInterrupt.h>
#include "Movement.h"
#include "Calib.h"
#include "Sensor.h"

bool activate = true;
//String commands;
//char commands;
String commands;

Movement *move;
Calib *cal;
Motor *motor;
Sensor *s;

bool DEBUG = true;

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
//  move->moveFront30CM();
  //  while (activate) {
  //    while (Serial.available() > 0) {
  //      commands = Serial.read();
  //      //      delay(100);
  //
  //      Serial.println("Received_command: " + String(commands));
  //      switch (commands) {
  //        case 'w': move->moveFront30CM(); break;
  //        case 's': move->moveReverse(Distance_10CM); break;
  //        case 'r': move->rotate(90); break;
  //        case 'f': activate = false; break;
  ////        case 'nb': move->newBatt(); break;
  //        default: break;
  //      }
  //      String output = "Received_command: " + String(commands);
  //      Serial.print(output);
  //    }
  //  }

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

bool executeCommand(String sub_command) {
  if (DEBUG) Serial.print("-Received Command: " + String(sub_command));
  if (sub_command.charAt(0) == 'a') {
    sub_command.remove(0, 1);
    int value = sub_command.toInt() + 1;
    Serial.println("a" + String(value));
  } else if (sub_command.charAt(0) == 'w') {
    move->moveFront30CM();
  } else if (sub_command.charAt(0) == 's') {

  }
}
