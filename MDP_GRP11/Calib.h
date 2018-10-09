#include "Sensor.h"
#include "Movement.h"

const bool DEBUG_CALIB = false;

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward();
    void calibForward(bool limitedFront);
    void calibForward(int *rFL, int *rFM, int *rFR, bool limitedFront);
    void calibWallF();
    void calibWallF(int *rFL, int *rFM, int *rFR);
    void calibWallL();
    void calibWallL(int *rLF, int *rLB);
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
