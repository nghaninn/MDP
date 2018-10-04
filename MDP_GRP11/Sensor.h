#include "Arduino.h"

#define ir1 A0 //FL
#define ir2 A1 //FM
#define ir3 A2 //FR
#define ir4 A3 //RF
#define ir5 A4 //LF
#define ir6 A5 //LB

static const int sFL_o []PROGMEM = {108, 223, 354, 479};
static const int sFM_o []PROGMEM = {84, 190, 330, 506};
static const int sFR_o []PROGMEM = {107, 215, 356, 527};
static const int sLF_o []PROGMEM = {121, 224, 342, 462};
static const int sLB_o []PROGMEM = {124, 237, 367, 479};
static const int sR_o  []PROGMEM = {188, 240, 343, 457, 576, 692};

//With 25% leeway of next distance
static const int sFL []PROGMEM = {137, 255, 384, 502};
static const int sFM []PROGMEM = {111, 217, 327, 440};
static const int sFR []PROGMEM = {134, 247, 380, 523};
static const int sLF []PROGMEM = {147, 252, 366, 483};
static const int sLB []PROGMEM = {152, 269, 392, 502};
static const int sR  []PROGMEM = {201, 265, 369, 485, 602, 716};

static const int sLF_Limit []PROGMEM = {79, 147}; //Lower limit, upper limit;
static const int sLB_Limit []PROGMEM = {84, 153};

static const int gShort []PROGMEM = {1, 2, 3, 4, 100};
static const int gLong  []PROGMEM = {1, 2, 3, 4, 5, 6, 100};

//static const int gShort []PROGMEM = {0, 1, 2, 3, 100};
//static const int gLong  []PROGMEM = {0, 1, 2, 3, 4, 5, 100};

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
  void printAllSensorsRAW();
};
