#include "Arduino.h"

#define ir1 A0 //FL
#define ir2 A1 //FM
#define ir3 A2 //FR
#define ir4 A3 //RF
#define ir5 A4 //LF
#define ir6 A5 //LB

static const int sFL_o []PROGMEM = {102, 214, 347, 479};
static const int sFM_o []PROGMEM = {79, 186, 313, 506};
static const int sFR_o []PROGMEM = {101, 211, 328, 454};
static const int sLF_o []PROGMEM = {105, 218, 334, 442};
static const int sLB_o []PROGMEM = {114, 236, 356, 445};
static const int sR_o  []PROGMEM = {185, 231, 331, 443, 567, 683};

//With 25% leeway of next distance
static const int sFL []PROGMEM = {130, 247, 378, 512};
static const int sFM []PROGMEM = {106, 213, 325, 455};
static const int sFR []PROGMEM = {128, 240, 357, 470};
static const int sLF []PROGMEM = {133, 246, 357, 469};
static const int sLB []PROGMEM = {145, 266, 376, 463};
static const int sR  []PROGMEM = {196, 256, 358, 473, 593, 708};

//static const int sFL []PROGMEM = {102, 214, 347, 479};
//static const int sFM []PROGMEM = {79, 186, 313, 506};
//static const int sFR []PROGMEM = {101, 211, 328, 454};
//static const int sLF []PROGMEM = {105, 218, 334, 442};
//static const int sLB []PROGMEM = {114, 236, 356, 445};
//static const int sR  []PROGMEM = {185, 231, 331, 443, 567, 683};

static const int sLF_Limit []PROGMEM = {71, 163, 93, 141}; // extreme Lower limit, extreme upper limit;
static const int sLB_Limit []PROGMEM = {70, 176, 96, 149};

static const int gShort []PROGMEM = {1, 2, 3, 4, 100};
static const int gLong  []PROGMEM = {2, 3, 4, 5, 6, 7, 100};

//static const int gShort []PROGMEM = {0, 1, 2, 3, 100};
//static const int gLong  []PROGMEM = {0, 1, 2, 3, 4, 5, 100};

static const int LB_Calib_Offset = 0;

const bool DEBUG_SENSOR = true;
const bool SENSOR_STABILITY_TEST = true;

class Sensor {
  private:
    int ir_val[10];
    int NB_SAMPLE = 10;
    int irDistance(int sensor);
    void merge(int arr[], int l, int m, int r);
    void mergeSort(int arr[], int l, int r);
    void readFrontLeftObstacle(int *oFL);
    void readFrontMidObstacle(int *oFM);
    void readFrontRightObstacle(int *oFR);
    void readLeftFrontObstacle(int *oLF);
    void readLeftBackObstacle(int *oLB);
    void readRightObstacle(int *oR);
    void readFrontLeftSensorRawValues(int *oFL);
    void readFrontMidSensorRawValues(int *oFM);
    void readFrontRightSensorRawValues(int *oFR);
    void readLeftFrontSensorRawValues(int *oLF);
    void readLeftBackSensorRawValues(int *oLB);
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
