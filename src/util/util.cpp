#include "main.h"

ExtendedPoint::ExtendedPoint(QLength ix, QLength iy, QAngle itheta) : theta(itheta)
{
    x = ix;
    y = iy;
}

/* ----------------------- Subtraction ----------------------- */
ExtendedPoint ExtendedPoint::operator-(const ExtendedPoint &ivec) // overloaded subtraction
{
    return ExtendedPoint(x - ivec.x, y - ivec.y, theta - ivec.theta);
}
ExtendedPoint ExtendedPoint::sub(const ExtendedPoint &ivec) // subtraction method
{
    return *this - ivec;
}

/* ------------------------- Addition ------------------------ */
ExtendedPoint ExtendedPoint::operator+(const ExtendedPoint &ivec) // overloaded addition
{
    return ExtendedPoint(x + ivec.x, y + ivec.y, theta + ivec.theta);
}
ExtendedPoint ExtendedPoint::add(const ExtendedPoint &ivec) // addition method
{
    return *this + ivec;
}

/* ---------------------- Multiplication --------------------- */
QLength ExtendedPoint::dot(const ExtendedPoint &ivec) // dot multiply vectors
{
    return (x.convert(inch) * ivec.x.convert(inch) + y.convert(inch) * ivec.y.convert(inch)) * inch;
}
ExtendedPoint ExtendedPoint::operator*(const double iscalar) // overloaded scalar multiplication
{
    return ExtendedPoint(x * iscalar, y * iscalar, theta * iscalar);
}
ExtendedPoint ExtendedPoint::scalarMult(const double iscalar) // multiply the vectors by a scalar
{
    return *this * iscalar;
}
ExtendedPoint ExtendedPoint::operator*(const ExtendedPoint &ivec) // elementwise multiplication
{
    return ExtendedPoint((x.convert(inch) * ivec.x.convert(inch)) * inch,
                         (y.convert(inch) * ivec.y.convert(inch)) * inch,
                         (theta.convert(degree) * ivec.theta.convert(degree)) * degree);
}
ExtendedPoint ExtendedPoint::eachMult(const ExtendedPoint &ivec) // elementwise multiplication
{
    return *this * ivec;
}

/* ----------------------- Comparative ----------------------- */
bool ExtendedPoint::operator==(const ExtendedPoint &ipoint) // overloaded equivalence check
{
    return x == ipoint.x && y == ipoint.y && theta == ipoint.theta;
}

/* -------------------------- Other -------------------------- */
QLength ExtendedPoint::dist(const ExtendedPoint &ivec) // distance between points
{
    return sqrt((x - ivec.x).convert(inch) * (x - ivec.x).convert(inch) +
                (y - ivec.y).convert(inch) * (y - ivec.y).convert(inch)) *
           inch;
}
QLength ExtendedPoint::mag() // magnitude
{
    return sqrt(x.convert(inch) * x.convert(inch) + y.convert(inch) * y.convert(inch)) * inch;
}
ExtendedPoint ExtendedPoint::normalize() // creates a vector with a length of 1
{
    return ExtendedPoint((x.convert(inch) / mag().convert(inch)) * inch,
                         (y.convert(inch) / mag().convert(inch)) * inch, theta);
}
std::string ExtendedPoint::string() // returns the point in string form for testing
{
    return "{" + std::to_string(x.convert(inch)).substr(0, 3) + ", " +
           std::to_string(y.convert(inch)).substr(0, 3) + ", " +
           std::to_string(theta.convert(degree)).substr(0, 3) + "}";
}

void waitForImu() // blocks the execution of the code until the imu is done calibrating
{
    while (def::imu_top.is_calibrating() || def::imu_side.is_calibrating())
        pros::delay(100);
}

bool waitUntil(std::function<bool()> icondition, QTime itimeout, std::function<void()> iaction)
{
    Timer timer = Timer();
    QTime startTime = timer.millis();

    while (!icondition())
    {
        if (itimeout != 0_ms && timer.millis() - startTime >= itimeout)
        {
            return false;
        }
        iaction();
        pros::delay(10);
    }
    return true;
}

double controlAdjust(double iinput,
                     double ipower) // adjusts the curve of the input from the joysticks
{
    double result = pow(iinput, ipower); // calculate the curve
    if (iinput * result < 0)             // if input is negative and output is positive
    {
        result = -result; // make it negative
    }
    return result;
}

void display_task_func(void *) // display task to be run independently
{
    while (true)
    {

        // room for any other miscellaneous debugging

        pros::delay(20);
    }
}

/* -------------------------- Macros ------------------------- */
void deploy() // deploys the robot
{
    pros::delay(250);
}

//--------------------------------------------------------------//
//--------------------------------------------------------------//
//--------------------------PID---------------------------------//
//--------------------------------------------------------------//
//--------------------------------------------------------------//
//--------------------------------------------------------------//
//--------------------------------------------------------------//

PID::PID(double ikP, double ikI, double ikD, double ikIRange, double isettlerError,
         double isettlerDerivative,
         QTime isettlerTime) // constructor that sets constants, and initializes variables
    : msettlerError(isettlerError), msettlerDerivative(isettlerDerivative),
      msettlerTime(isettlerTime), msettler(TimeUtilFactory::withSettledUtilParams(
                                               msettlerError, 
                                               msettlerDerivative, 
                                               msettlerTime)
                                               .getSettledUtil()),
      mkP(ikP), mkI(ikI), mkD(ikD), mkIRange(ikIRange), merror(0), mlastError(0), mtotalError(0),
      mderivative(0)
{
}

PID::PID(const PID &iother) // copy constructor for duplicating PID objects behind the scenes
{
    msettlerError = iother.msettlerError;
    msettlerDerivative = iother.msettlerDerivative;
    msettlerTime = iother.msettlerTime;
    msettler =
        TimeUtilFactory::withSettledUtilParams(msettlerError,
                             msettlerDerivative, 
                             msettlerTime)
            .getSettledUtil();
    mkP = iother.mkP;
    mkI = iother.mkI;
    mkD = iother.mkD;
    mkIRange = iother.mkIRange;
    mlastError = iother.mlastError;
    mtotalError = iother.mtotalError;
    mderivative = iother.mderivative;
}

double PID::getLastError() { return mlastError; }
double PID::getTotalError() { return mtotalError; }

void PID::setGains(double ikP, double ikI, double ikD) // used only for changing constants later
{
    mkP = ikP;
    mkI = ikI;
    mkD = ikD;
}

double PID::getP() { return mkP; }
double PID::getI() { return mkI; }
double PID::getD() { return mkD; }

double PID::iterate(double ierror) // goes through one iteration of the PID loop
{
    merror = ierror;
    if (mkI != 0) // regulate integral term
    {
        if (abs(merror) < mkIRange && merror != 0) // if in range, update mtotalError
        {
            mtotalError += merror;
        }
        else
        {
            mtotalError = 0;
        }
        util::chop<double>(-50 / mkI, 50 / mkI,
                           mtotalError); // limit mtotalError to prevent integral windup
    }

    mderivative = merror - mlastError; // calculate the derivative before lastError is overwritten
    mlastError = merror;               // save the current error for the next cycle

    return merror *
               mkP +
           mtotalError * mkI + mderivative * mkD;
}

bool PID::isSettled() // returns whether or not the controller is settled at the target
{
    return msettler->isSettled(merror);
}

/* ---------------------- Angle Wrappers ---------------------  /
 * These methods take any angle, and return an angle representing the same position in a specific
 * range. For example, wrapDeg(370) returns 10, because 370 is out of the range [0, 360).
 */
double util::wrapDeg(double iangle) // range [0, 360)
{
    iangle = fmod(iangle, 360);
    if (iangle < 0)
        iangle += 360;
    return iangle;
}
double util::wrapDeg180(double iangle) // range [-180, 180]
{
    iangle = fmod(iangle, 360);
    if (iangle < -180)
        iangle += 360;
    else if (iangle > 180)
        iangle -= 360;
    return iangle;
}
double util::wrapRad(double iangle) // range [0, 2pi)
{
    iangle = fmod(iangle, 2 * 3.14159265358979323846);
    if (iangle < 0)
        iangle += 2 * 3.14159265358979323846;
    return iangle;
}
double util::wrapRadPI(double iangle) // range [-pi, pi]
{
    iangle = fmod(iangle, 2 * 3.14159265358979323846);
    if (iangle < -3.14159265358979323846)
        iangle += 2 * 3.14159265358979323846;
    else if (iangle > 3.14159265358979323846)
        iangle -= 2 * 3.14159265358979323846;
    return iangle;
}
QAngle util::wrapQAngle(QAngle iangle) // range [0, 360) for QAngles
{
    iangle = fmod(iangle.convert(degree), 360) * degree;
    if (iangle < 0_deg)
        iangle += 360_deg;
    return iangle;
}
QAngle util::wrapQAngle180(QAngle iangle) // range [-180, 180] for QAngles
{
    iangle = fmod(iangle.convert(degree), 360) * degree;
    if (iangle < -180_deg)
        iangle += 360_deg;
    else if (iangle > 180_deg)
        iangle -= 360_deg;
    return iangle;
}

/* ------------------- Other Angle Methods ------------------- */
bool util::degIsBetween(double istart, double iend,
                        double icheck) // checks if an angle is between 2 others
{
    iend -= istart;
    if (iend < 0)
    {
        iend += 360;
    }
    icheck -= istart;
    if (icheck < 0)
    {
        icheck += 360;
    }
    return icheck < iend;
}
