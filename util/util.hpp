#pragma once
#include "main.h"

struct ExtendedPoint : Point
{
    ExtendedPoint(QLength ix, QLength iy, QAngle itheta);

    QAngle theta{0_deg}; // stores the orientation at the point, with a default of 0 degrees

    /* ----------------------- Subtraction ----------------------- */
    ExtendedPoint operator-(const ExtendedPoint &ivec);
    ExtendedPoint sub(const ExtendedPoint &ivec);

    /* ------------------------- Addition ------------------------ */
    ExtendedPoint operator+(const ExtendedPoint &ivec);
    ExtendedPoint add(const ExtendedPoint &ivec);

    /* ---------------------- Multipliation ---------------------- */
    QLength dot(const ExtendedPoint &ivec); // dot multiply the vectors
    ExtendedPoint operator*(const double iscalar);
    ExtendedPoint scalarMult(const double iscalar);     // multiply the vectors by a scalar
    ExtendedPoint operator*(const ExtendedPoint &ivec); // elementwise multiplication
    ExtendedPoint eachMult(const ExtendedPoint &ivec);

    /* ----------------------- Comparative ----------------------- */
    bool operator==(const ExtendedPoint &ipoint); // checks to see of the points are the same

    /* -------------------------- Other -------------------------- */
    QLength dist(const ExtendedPoint &ivec); // distance between points
    QLength mag();                           // magnitude
    ExtendedPoint normalize();               // creates a vector with a length of 1
    std::string string();
};

void waitForImu(); // blocks execution of the code until the imu is done calibrating

bool waitUntil(
    std::function<bool()> icondition, okapi::QTime itimout = 0_ms,
    std::function<void()> iaction = []() {});             // runs code until the condition is true
double controlAdjust(double iinput, double ipower = 1);   // adjusts the curve of the input from the joysticks

void display_task_func(void *); // display task to be run independently

void deploy(); // deploys the robot

//----------------------------------------------PID---------------------------------//
class PID
{
    double msettlerError, msettlerDerivative; // target error and derivative for the settler
    QTime msettlerTime;                       // target time for the settler
    std::unique_ptr<SettledUtil> msettler;    // okapi settler that is used to determine if the PID
                                              // should stop, based on error, derivative, and time

    double mkP, mkI, mkD, mkIRange; // constants

    double merror, mlastError, mtotalError;
    double mderivative; // used for storing derivative before lastError is overwritten

public:
    PID(double ikP, double ikI, double ikD, double ikIRange, double isettlerError,
        double isettlerDerivative, QTime isettlerTime); // constructor

    PID(const PID &iother); // copy constructor

    double getLastError();
    double getTotalError();

    void setGains(double ikP, double ikI, double ikD);

    double getP();
    double getI();
    double getD();

    double iterate(double ierror); // goes through one iteration of the PID loop

    bool isSettled(); // returns whether or not the controller is settled at the target
};

namespace util
{
    /* ---------------------- Angle Wrappers ---------------------  /
     * All of these functions take an angle as an input, and return an
     * angle fitting into a certain range. For example, wrapDeg(370) would
     * return 10, and wrapDeg180(200) would return -160
     */
    double wrapDeg(double iangle);       // [0, 360)
    double wrapDeg180(double iangle);    // [-180, 180)
    double wrapRad(double iangle);       // [0, 2pi)
    double wrapRadPI(double iangle);     // [-pi, pi)
    QAngle wrapQAngle(QAngle iangle);    // [0_deg, 360_deg)
    QAngle wrapQAngle180(QAngle iangle); // [-180_deg, 180_deg)
    /* ------------------- Other Angle Methods ------------------- */
    bool degIsBetween(double istart, double iend,
                      double icheck); // checks if an angle is between 2 others
    /* ------------------------- Find Max ------------------------  /
 * these functions all find the maximum value of a few different types
 * of inputs. They use templates so they can be used on different types,
 * and on arrays of different lengths.
 */
template <class T, std::size_t N>
T findMax(const std::array<T, N> && iarray) // returns the max value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (val > largest)
            largest = val; // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findMax(const std::array<T, N> & iarray) // returns the max value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (val > largest)
            largest = val; // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findAbsMax(const std::array<T, N> && iarray) // returns the max absolute value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (abs(val) > largest)
            largest = abs(val); // stores the largest value
    return largest;
}
template <class T, std::size_t N>
T findAbsMax(const std::array<T, N> & iarray) // returns the max absolute value in iarray
{
    T largest = iarray.at(0);    // gives largest a value to compare with
    for (const T & val : iarray) // loops through all values
        if (abs(val) > largest)
            largest = abs(val); // stores the largest value
    return largest;
}

/* ------------------------- Fitters -------------------------  /
 * These functions modfify the input to fit in a specified range
 */
template <std::size_t N>
std::array<double, N> scaleToFit(double imagnitude, std::array<double, N> && iarray) // scales all elements in iarray to fit within [-imagnitude, imagnitude]
{
    double largest = findAbsMax<double, N>(iarray);
    if (largest > imagnitude) // if anything is out of range
    {
        largest = std::abs(largest);
        for (double & val : iarray) // scales everything down to fit in the range
            val = val / largest * imagnitude;
    }
    return iarray;
}
template <std::size_t N>
void scaleToFit(double imagnitude, std::array<double, N> & iarray) // scales all elements in iarray to fit within [-imagnitude, imagnitude]
{
    double largest = findAbsMax<double, N>(iarray);
    if (largest > imagnitude) // if anything is out of range
    {
        largest = std::abs(largest);
        for (double & val : iarray) // scales everything down to fit in the range
            val = val / largest * imagnitude;
    }
}

template <class T, std::size_t N>
void chop(T imin, T imax, std::array<T, N> & iarray) // if any values in iarray are out of range, they are set to the limit
{
    for (double & val : iarray)
    {
        if (val > imax)
            val = imax;
        else if (val < imin)
            val = imin;
    }
}
template <class T>
void chop(T imin, T imax, T & inum) // if the value is out of range, it is set to the limit
{
    if (inum > imax)
        inum = imax;
    else if (inum < imin)
        inum = imin;
}
} // namespace util

