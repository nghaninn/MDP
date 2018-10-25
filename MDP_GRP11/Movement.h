#include "Motor.h"

static const int Distance_5CM = 140;
const int d10 = 275;//245;//278;
const int d20 = 583;
const int d30 = 888;
const int d40 = 1193;
const int d50 = 1495;
const int d60 = 1799;
const int d70 = 2104;
const int d80 = 2410;
const int d90 = 2714;
const int d100 = 3020;
const int d110 = 3324;
const int d120 = 3608;
const int d130 = 3912;
const int d140 = 4216;
const int d150 = 4518;
const int d160 = 4820;
const int d170 = 5125;

const int d180 = d170;//4860;//5004;//2500;//
static const int Distance_MAX = d120;
const int d180_stop = 5004;

static const int Rotate_45deg = 185;
const int Rotate_90deg = 395;//385;//357;
const int L_offset = -1;//385;//357;
const int Rotate_180deg = 793;//Rotate_90deg * 2.057;
const int Rotate_270deg = Rotate_90deg * 3.3;
const int Rotate_360deg = Rotate_90deg * 4.51;

const int MSpeed = 350;
const int MSpeedR = 300;
const int MSpeedC = 150;
const int offset = 0;//-10; //M1 Speed offset (Going abt 20pwm faster)

const int delayTime = 10;

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
    int computePID_left(int angle);
    void moveSmall(int distance);
    void rotateSmall(int distance);
};
