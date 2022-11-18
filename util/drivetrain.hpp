#pragma once
#include "AsyncAction.hpp"
#include "main.h"


class Drivetrain
{
private:
// --------------------- Motor References
    static Motor & mmtrLeftFront;
    static Motor & mmtrRightFront;
    static Motor & mmtrRightBack;
    static Motor & mmtrLeftBack;

//------------------------- Chassis Builder
    static std::shared_ptr<ChassisController>
        mchassis; // chassis object for using Pathfilder through okapi

protected:
  // ------------------------- Settings ------------------------ 
    static double mmaxSpeed;
    static bool menabled;
    /* -------------------- Odometry Accessors ------------------- */
    static OdomState getState(); // get position as OdomState
    static QLength getXPos();
    static QLength getYPos();
    static QAngle getTheta();
    static ExtendedPoint getPoint(); // get position as ExtendedPoint

    static QAngle
    angleToPoint(const Point &itargetPoint);
public:

// ----------------------- Getter And Setter Methods
    static double getMaxSpeed();
    static void setMaxSpeed(const double imaxSpeed);

    static bool isEnabled();
    static void enable(); // allows movements to be startable
    static void disable(); // stops active movements

    static void
    checkNextAsync(const double & ierror,
                   std::vector<AsyncAction> &
                       iactions); // checks if the next AsyncAction should execute, and executes it
                                  // (and removes it from the list) if it should

    static void lock();
    static void unlock();

    static void moveIndependant(
        double ileft, double iright, const bool idesaturate = true); // moves each motor {lf, rf, rb, lb} in range [-1,1]
    static void
    moveTank(const double ileft, const double iright,
             const bool idesaturate =
                 true); // spins the left side and right side motors at certian speeds [-1,1]
    static void moveArcade(
        const double iforward, const double iturn, const bool idesaturate = true);   // moves the robot with arcade-style inputs (range[-1,1])
    static void arcadeFor(const double iforward, const double iturn, const int ims); // move arcade for time

static void
    turnToAngle(QAngle iangle, std::vector<AsyncAction> iactions = {},
                PID ipid = PID(.02, 0.01, 0.2, 1, 0.5, 0.01, 1_ms)); // turns to an angle using set PID gains, and
                                                                       // executing the AsyncActions at the right times

    static void straightForDistance(QLength idistance, std::vector<AsyncAction> iactions = {},
    PID imagnitudePID = PID(.069, 0.0,0.01, 0, 0.5, 0.001, 1_ms),
    PID iturnPID = PID(0.69, 0.0, 0.0, 0, 0.25, 0.01, 1_ms));

static void straightToPoint(
        ExtendedPoint itarget,
        std::vector<AsyncAction> iactions = {}, bool inoReverse = false, 
        QLength inoTurnRange = 6_in, 
        double iturnWeight = 1.7, 
        PID imagnitudePID = PID(0.2, 0.001, 0.0, 3.0, 0.5, 0.00001, 1_ms), 
        PID iturnPID = PID(0.07, 0.02, 0.20, 1, 0.25, 0.01, 1_ms)); // drives to the point without strafing using set PID/Slew gains, and executing the AsyncActions at the right times

    static std::shared_ptr<AsyncMotionProfileController>
        mprofiler;
}; // End of the Drivetrain Class

namespace def
{
extern Drivetrain drivetrain; // declares the drivetrain object as external, to only use the drivetrain once
                                // when we use the drivetain class 
}