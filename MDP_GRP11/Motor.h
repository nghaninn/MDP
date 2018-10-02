#include <DualVNH5019MotorShield.h>

//motors pin
static volatile long M1Ticks;
static volatile long M2Ticks;

static DualVNH5019MotorShield md;

class Motor {
  private:
    void init();
  public:
    static const int M1A = 3;
    static const int M1B = 5;
    static const int M2A = 11;
    static const int M2B = 13;
    Motor();
    void encoder1();
    void encoder2();
    long getM1Ticks();
    long getM2Ticks();
    void resetTicks();
};
