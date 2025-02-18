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

static const int sLF_Limit []PROGMEM = {71, 163, 93, 128}; // extreme Lower limit, extreme upper limit;
static const int sLB_Limit []PROGMEM = {70, 176, 96, 128};

static const int gShort []PROGMEM = {1, 2, 3, 4, 100};
static const int gLong  []PROGMEM = {2, 3, 4, 5, 6, 7, 100};

static const int LB_Calib_Offset = 0;

//int rFL, rFM, rFR, rLF, rLB, rR;
//int oFL, oFM, oFR, oLF, oLB, oR;
//int aFL[3], aFM[3], aFR[3], aLF[3], aLB[3], aR[3];
//int bFL[3], bFM[3], bFR[3], bLF[3], bLB[3], bR[3];

const bool SENSOR_STABILITY_TEST = true;
static int NB_SAMPLE = 10;

class Sensor {
  private:
    int ir_val[10];
    int irDistance(int sensor);
    void merge(int arr[], int l, int m, int r);
    void mergeSort(int arr[], int l, int r);
    void readFrontLeftObstacle();
    void readFrontMidObstacle();
    void readFrontRightObstacle();
    void readLeftFrontObstacle();
    void readLeftBackObstacle();
    void readRightObstacle();
    void readFrontLeftSensorRawValues();
    void readFrontMidSensorRawValues();
    void readFrontRightSensorRawValues();
    void readLeftFrontSensorRawValues();
    void readLeftBackSensorRawValues();
  public:
    Sensor();
    void detect();
    void detectAll();
    void readObstacle();
    void readSensorRawValues();
    void readFrontSensorRawValues();
    void readLeftSensorRawValues();
    void readRightSensorRawValues();
    bool hasObstacleForCalib();
    int hasObstacleForSelfCalib();
    void printAllSensors();
    void printAllSensorsRAW();
};
