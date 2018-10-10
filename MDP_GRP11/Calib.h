#include "Movement.h"
#include "Sensor.h"

const bool DEBUG_CALIB = true;

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward();
    void calibForward(bool limitedFront);
    void calibWallF();
    void calibWallL();
    void calibDelay();
  public:
    bool isCalibrating;
    Calib();
    Movement *mov;
    void needSelfCalib();
    void selfCalib();
    void selfCalib(bool multipleLevel);
    void calib();
    void calibFront();
    void calibLeft();
};
