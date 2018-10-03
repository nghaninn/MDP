#include "Arduino.h"

#define ir1 A0 //FL
#define ir2 A1 //FM
#define ir3 A2 //FR
#define ir4 A3 //RF
#define ir5 A4 //LF
#define ir6 A5 //LB

static const int sFL []PROGMEM = {99, 213, 344, 479};
static const int sFM []PROGMEM = {76, 181, 313, 506};
static const int sFR []PROGMEM = {98, 207, 347, 506};
static const int sLF []PROGMEM = {101, 214, 328, 449};
static const int sLB []PROGMEM = {110, 232, 354, 454};
static const int sR  []PROGMEM = {188, 234, 335, 449, 573, 705};

static const int gShort []PROGMEM = {1, 2, 3, 4, 100};
static const int gLong  []PROGMEM = {1, 2, 3, 4, 5, 6, 100};

static const int LB_Calib_Offset = -5;

const int NB_SAMPLE = 10;
const bool DEBUG_SENSOR = false;

class Sensor {
private:
  int ir_val[10];
  int irDistance(int sensor);
  void merge(int arr[], int l, int m, int r);
  void mergeSort(int arr[], int l, int r);
public:
  Sensor();
  void detect();
  void detectAll();
  void readObstacle(int *oF, int *oL, int *oR);
  void readObstacle(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR);
  void readSensorRawValues(int *oFL, int *oFM, int *oFR, int *oLF, int *oLB, int *oR);
  void readFrontSensorRawValues(int *oFL, int *oFM, int *oFR);
  void readLeftSensorRawValues(int *oLF, int *oLB);
  void readRightSensorRawValues(int *oR);
  bool hasObstacleForCalib(int *rFL, int *rFM, int *rFR, int *rLF, int *rLB, int *rR);
  int hasObstacleForSelfCalib(int *rLF, int *rLB);
  void printAllSensors();
};
