#include "Arduino.h"

#define ir1 A0 //FL
#define ir2 A1 //FM
#define ir3 A2 //FR
#define ir4 A3 //RF
#define ir5 A4 //LF
#define ir6 A5 //LB

const int rFL []PROGMEM = {99, 213, 344, 479};
const int rFM []PROGMEM = {76, 181, 313, 506};
const int rFR []PROGMEM = {98, 207, 347, 506};
const int rLF []PROGMEM = {101, 214, 328, 449};
const int rLB []PROGMEM = {110, 232, 354, 454};
const int rR  []PROGMEM = {188, 234, 335, 449, 573, 705};

class Sensor {
  private:
    const int NB_SAMPLE = 10;
    const bool DEBUG_SENSOR = true;
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
};
