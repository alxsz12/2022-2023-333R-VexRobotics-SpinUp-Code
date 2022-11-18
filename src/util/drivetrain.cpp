#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "util.hpp"

//------------------------Motor References-----------------//
Motor & Drivetrain::mmtrLeftFront = def::mtr_dt_left_front;
Motor & Drivetrain::mmtrRightFront = def::mtr_dt_right_front;
Motor & Drivetrain::mmtrRightBack = def::mtr_dt_right_back;
Motor & Drivetrain::mmtrLeftBack = def::mtr_dt_left_back;

// ----------------------Okapi Chassis---------------------//
std::shared_ptr<ChassisController> Drivetrain::mchassis =
    ChassisControllerBuilder()
        .withMotors({Drivetrain::mmtrLeftFront, Drivetrain::mmtrLeftBack},
                    {Drivetrain::mmtrRightFront, Drivetrain::mmtrRightBack})
        .withDimensions(AbstractMotor::gearset::blue,
                        {{def::DRIVE_WHEEL_DIAMETER, def::DRIVE_OFFSET}, imev5BlueTPR})
        .build(); // chassis object for using Pathfilder through okapi
// Variables
double Drivetrain::mmaxSpeed = def::SET_DT_MAX;
bool Drivetrain::menabled = true;

std::shared_ptr<AsyncMotionProfileController> Drivetrain::mprofiler =
    AsyncMotionProfileControllerBuilder()
        .withLimits({def::DRIVE_MAX_SPEED.convert(mps), def::DRIVE_MAX_ACCEL.convert(mps2), 8.0})
        .withOutput(Drivetrain::mchassis)
        .buildMotionProfileController();

double Drivetrain::getMaxSpeed() { return mmaxSpeed; }
void Drivetrain::setMaxSpeed(double imaxSpeed) { mmaxSpeed = imaxSpeed; }

bool Drivetrain::isEnabled() { return menabled; }
/* -------------------- Odometry Accessors ------------------- */
OdomState Drivetrain::getState() // get position as OdomState
{
    return CustomOdometry::getState();
}
QLength Drivetrain::getXPos() { return CustomOdometry::getX(); }
QLength Drivetrain::getYPos() { return CustomOdometry::getY(); }
QAngle Drivetrain::getTheta() { return CustomOdometry::getTheta(); }

ExtendedPoint Drivetrain::getPoint() // get position as ExtendedPoint
{
    return ExtendedPoint(getXPos(), getYPos(), getTheta());
}
QAngle Drivetrain::angleToPoint(
    const Point &itargetPoint) // calculates the field centric direction to the itargetPoint from
                               // the robot's current position
{
    return (atan((getYPos().convert(inch) - itargetPoint.y.convert(inch)) /
                 (getXPos().convert(inch) - itargetPoint.x.convert(inch))) +
            (getXPos() > itargetPoint.x ? M_PI : 0)) *
           radian;
}
// ---------------------Enabler Functions-------------------------//
void Drivetrain::enable() // allows movements to be startable
{
    menabled = true;
}

void Drivetrain::disable() // stops active movemnts
{
    menabled = false;
    moveTank(0, 0, false);
}


//----------------------Async Function-------------------------//
void Drivetrain::checkNextAsync(
    const double & ierror,
    std::vector<AsyncAction> & iactions) // checks if the next AsyncAction should execute, and
                                         // executes it (and removes it from the list) if it should
{
    if (iactions.size()) // if there is at least one action to execute
    {
        const AsyncAction & nextAction = iactions.at(0);
        if (ierror < nextAction.merror) // if the robot is close enough to the target
        {
            nextAction.maction(); // execute the action
            iactions.erase(iactions.begin()); // remove the action, having already executed it
        }
    }
}

//--------------------Lock & Unlocking Functions -------------------//

void Drivetrain::lock()
{
    mmtrLeftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    mmtrRightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    mmtrRightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
    mmtrLeftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
}
void Drivetrain::unlock()
{
    mmtrLeftFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    mmtrRightFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    mmtrRightBack.setBrakeMode(AbstractMotor::brakeMode::coast);
    mmtrLeftBack.setBrakeMode(AbstractMotor::brakeMode::coast);
}

void Drivetrain::moveIndependant(
    double ileft, double iright, const bool idesaturate) // moves each motor {lf, rf, rb, lb} in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 2> motor_values =
            util::scaleToFit<2>(mmaxSpeed, {ileft, iright});
        ileft = motor_values[0];
        iright = motor_values[1];
    }
    else
    {
        util::chop<double>(-1, 1, ileft);
        util::chop<double>(-1, 1, iright);
    }
    // moves all of the motors by voltage
    mmtrLeftFront.moveVoltage(12000 * ileft);
    mmtrRightFront.moveVoltage(12000 * iright);
    mmtrRightBack.moveVoltage(12000 * iright);
    mmtrLeftBack.moveVoltage(12000 * ileft);
}
void Drivetrain::moveTank(double ileft, double iright,
                          const bool idesaturate) // spins the left side and right side motors at
                                                  // certian speeds in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 2> motor_values = util::scaleToFit<2>(mmaxSpeed, {ileft, iright});
        ileft = motor_values[0];
        iright = motor_values[1];
    }
    Drivetrain::moveIndependant(
        ileft, iright,
        false); // don't try to desaturate, because the values have already been desaturated
}
void Drivetrain::moveArcade(
    double iforward, double iturn,
    const bool idesaturate) // moves the robot with arcade-style inputs in range [-1,1]
{
    if (idesaturate) // desaturates values
    {
        std::array<double, 2> motor_values = {iforward + iturn, iforward - iturn};
        util::scaleToFit<2>(mmaxSpeed, motor_values);                  // modifies reference to motor_values
        Drivetrain::moveIndependant(motor_values[0], motor_values[1]); // moves the motors from within the if to
                                                                       // prevent the need to copy values
    }
    else
    {
        Drivetrain::moveIndependant(iforward + iturn, iforward - iturn, false); // don't desaturate
    }
}

void Drivetrain::arcadeFor(const double iforward, const double iturn, const int ims)
{
    Drivetrain::moveArcade(iforward, iturn);
    pros::delay(ims);
    Drivetrain::moveArcade(0, 0, false);
}

//------------------------------------------------------------------//
void Drivetrain::turnToAngle(QAngle iangle, std::vector<AsyncAction> iactions, PID ipid)
{
    enable(); // make sure the action can run
    while (menabled && !ipid.isSettled())
    {
        double angleError = util::wrapDeg180((iangle - Drivetrain::getTheta()).convert(degree)); // calculates the angle the robot needs to turn

        Drivetrain::moveArcade(0, -ipid.iterate(angleError), false);
        // std::cout << "in angle loop. Error: " << angleError << "     use: " << counter << std::endl;
        pros::delay(20);
    }
    Drivetrain::moveArcade(0, 0, false);
    def::controller.rumble("-");
}

void Drivetrain::straightForDistance(QLength idistance, std::vector<AsyncAction> iactions,
                                     PID imagnitudePID, PID iturnPID)
{
    double inStart = mmtrLeftFront.getPosition() * def::DRIVE_DEG_TO_IN; // starting distance of the motors
    QAngle startAngle = Drivetrain::getTheta();                          // starting angle of the robot

    enable();
    while (menabled && !imagnitudePID.isSettled())
    {
        double angleError = util::wrapDeg180((startAngle - Drivetrain::getTheta()).convert(degree)); // calculates the angle the robot needs to turn
        double inError = idistance.convert(inch) -
                         mmtrLeftFront.getPosition() * def::DRIVE_DEG_TO_IN +
                         inStart; // calculates how far the robot needs to drive

        Drivetrain::checkNextAsync(
            inError,
            iactions); // executes the next action if availible, and removes it from the list

        Drivetrain::moveArcade(imagnitudePID.iterate(inError), -iturnPID.iterate(angleError), false);

        pros::delay(20);
    }
    Drivetrain::moveArcade(0, 0, false);
    def::controller.rumble("-");
}

void Drivetrain::straightToPoint(
    ExtendedPoint itarget, std::vector<AsyncAction> iactions, bool inoReverse, QLength inoTurnRange,
    double iturnWeight, PID imagnitudePID, PID iturnPID) // drives to the point without strafing using set PID/Slew gains, and executing
                    // the AsyncActions at the right times
{
    const double noTurnRangeIn = inoTurnRange.convert(inch);

    enable(); // make sure the action can run
    while (menabled && !imagnitudePID.isSettled())
    {
        QAngle angleToPoint = util::wrapQAngle180(
            Drivetrain::angleToPoint(itarget) -
            Drivetrain::getTheta()); // how much the robot needs to turn to face the point
        double inToPoint = OdomMath::computeDistanceToPoint(itarget, Drivetrain::getState())
                               .convert(inch); // how far the robot is away from the target
        double inForward =
            inToPoint *
            cos(angleToPoint.convert(radian)); // how far the robot needs to drive straight (no
                                               // turning) to get as close to the target as possible

        double forward = 
            imagnitudePID.iterate(inForward); // calculates value from PID fed into Slew
        util::chop<double>(-mmaxSpeed, mmaxSpeed,
                           forward); // limits the values in [-mmaxSpeed, mmaxSpeed]

        double turn;
        if (inToPoint > noTurnRangeIn) // if the robot is far enough away from the target
        {
            if (!inoReverse) // not tested (1/10/2022)
            {
                QAngle old = angleToPoint;
                if (angleToPoint > 90_deg)
                    angleToPoint -= 180_deg;
                else if (angleToPoint < -90_deg)
                    angleToPoint += 180_deg;
                // std::cout << "old: " << old.convert(degree) << "new: " << angleToPoint.convert(degree) << std::endl;
            }
            turn = iturnPID.iterate(
                angleToPoint.convert(degree)); // calculates value from PID fed into Slew
            util::chop<double>(-mmaxSpeed, mmaxSpeed, turn);
        }
        else
        {
            turn = 0; // don't turn when too close to the target
        }

        if (abs(turn) == mmaxSpeed) // if the robot is turning at max speed (which means it must be
                                    // far off target)
        {
            turn *= iturnWeight; // increase the amount to turn, so that it turns faster as a result
                                 // of forward getting scaled down in moveArcade
        }

        Drivetrain::checkNextAsync(
            inToPoint,
            iactions); // executes the next action if availible, and removes it from the list

        // std::cout << "forward: " << forward << "     turn: " << turn << std::endl;

        Drivetrain::moveArcade(forward, -turn, true);

        pros::delay(20);
    }
    Drivetrain::moveArcade(0, 0, false);
}