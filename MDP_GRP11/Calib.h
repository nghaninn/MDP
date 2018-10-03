#include "Sensor.h"
#include "Movement.h"

const bool DEBUG_CALIB = true;

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward();
    void calibForward(int *rFL, int *rFM, int *rFR);
    void calibWallF();
    void calibWallF(int *rFL, int *rFM, int *rFR);
    void calibWallL();
    void calibWallL(int *rLF, int *rLB);
  public:
    Calib();
    Movement *mov;
    void needSelfCalib();
    void selfCalib();
    void calib();
};
