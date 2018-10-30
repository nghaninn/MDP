#include "Movement.h"
#include "Sensor.h"

static int SELF_LEFTWALL_CALIB_COUNTER;

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward();
    void calibForward(bool limitedFront);
    void calibForward(bool limitedFront, bool onlyMid);
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
    void selfCalib(bool usingFront);
    void selfCalibFront();
    void calib();
    void calibFront();
    void calibFrontLeft();
    void calibFrontRight();
    void calibLeft();
};
