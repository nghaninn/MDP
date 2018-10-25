#include "Movement.h"
#include "Sensor.h"

static int SELF_LEFTWALL_CALIB_COUNTER;

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward();
    void calibForward(bool limitedFront);
    void calibWallF();
    void calibWallL();
    void calibWallFront_L();
    void calibWallFront_R();
    void calibDelay();
  public:
    bool isCalibrating;
    Calib();
    Movement *mov;
    void selfCalib();
    void selfCalib(bool multipleLevel);
    void selfCalibFront();
    void calib();
    void calibFront();
    void calibLeft();
};
