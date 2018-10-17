
extern int rFL, rFM, rFR, rLF, rLB, rR;
extern int oFL, oFM, oFR, oLF, oLB, oR;
extern int aFL[3], aFM[3], aFR[3], aLF[3], aLB[3], aR[3];
extern int bFL[3], bFM[3], bFR[3], bLF[3], bLB[3], bR[3];

static const bool DEBUG = false;
static const bool DEBUG_SENSOR = false;
static const bool DEBUG_MOVEMENT = false;
static const bool DEBUG_CALIB = false;

static bool AUTO_SELF_CALIB = true;
static bool CALIB_SENSORS_PRINTVALUES = false;
static int ALREADY_SENT_OUT_SENSOR = 0;
static int LEFT_CAL_COUNT = 0;
