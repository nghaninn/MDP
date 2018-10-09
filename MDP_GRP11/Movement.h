#include "Motor.h"

static const int Distance_5CM = 140;
const int Distance_10CM = 275;//245;//278;
const int Distance_20CM = Distance_10CM * 2.12;
const int Distance_30CM = Distance_10CM * 3.23;
static const int Distance_MAX = 3575;
const int d180 = 4750;//5004;//2500;//
const int d180_stop = 5004;

static const int Rotate_45deg = 185;
const int Rotate_90deg = 390;//385;//357;
const int L_offset = 1;//385;//357;
const int Rotate_180deg = 793;//Rotate_90deg * 2.057;
const int Rotate_270deg = Rotate_90deg * 3.3;
const int Rotate_360deg = Rotate_90deg * 4.51;

const int MSpeed = 250;
const int MSpeedR = 150;
const int MSpeedC = 150;
const int offset = 0;//-10; //M1 Speed offset (Going abt 20pwm faster)

const bool DEBUG_MOVEMENT = false;

class Movement {
  private:
    int integral;
    long PrevTicks;
    Motor *motor;
    void brake();
  public:
    Movement();
    void newBatt();
    void move(int forward);
    void rotate(int degree);
    void rotateL(int degree);
    void rotateR(int degree);
    void rotate(int degree, bool isRight);
    void rotate(int degree, double distance, bool isRight);
    void moveFrontShort(int distance);
    void moveFront(double distance);
    void moveReverse(double distance);
    void moveFront5CM();
    void moveFront10CM();
    void moveFront20CM();
    void moveFront30CM();
    double computePID();
    double computePIDShort();
    int computePID_right(int angle);
    void moveSmall(int distance);
    void rotateSmall(int distance);
};
