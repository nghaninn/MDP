#include "Sensor.h"
#include "Movement.h"

class Calib {
  private:
    bool selfCalibNeeded;
    Sensor *sensor;
    void calibForward(int *rFL, int *rFM, int *rFR);
    void calibWallF(int *rFL, int *rFR);
    void calibWallL();
  public:
    Calib();
    Movement *mov;
    void needSelfCalib();
    void selfCalib();
    void calib();
};
