#pragma once
#include "main.h"
#include "pros/adi.hpp"

#define DT_STATES DrivetrainStateMachine::MStates
#define FW_STATES FlywheelStateMachine::MStates
#define INTAKE_STATES IntakeStateMachine::MStates
#define INDEXER_STATES IndexerStateMachine::MStates
#define END_STATES EndGameStateMachine::MStates



// Async Functions
#define makeFunc(i) [&]() i
#define cutDrive(i)                                                                                \
    {                                                                                              \
        AsyncAction(i, makeFunc({ def::drivetrain.disable(); }))                                   \
    }


namespace def
{


//--------------------------------------------------------------------//
//-----------------------------MOTORS---------------------------------//
//--------------------------------------------------------------------//

//-----------Drivetrain Motors
extern Motor mtr_dt_left_front;
extern Motor mtr_dt_right_front;
extern Motor mtr_dt_left_back;
extern Motor mtr_dt_right_back;

//-----------Indexer Motor-----
extern Motor mtr_ix;

//-----------Intake Motors------
extern Motor mtr_it_m;

//Flywheel Motors
extern Motor mtr_fw_fw1;
extern Motor mtr_fw_fw2;


//Pnumatics
extern pros::ADIDigitalOut sol_endg;
//--------------------------------------------------------------------//
//-----------------------------ODOM-----------------------------------//
//--------------------------------------------------------------------//

//-----------Odometry Encoders
extern ADIEncoder track_encoder_side;
//extern ADIEncoder track_encoder_forward;

//--------------------------------------------------------------------//
//-----------------------------SENSORS--------------------------------//
//--------------------------------------------------------------------//

//-----------Inertial Sensors
extern pros::Imu imu_top;
extern pros::Imu imu_side;

//Rotation Sensors
extern RotationSensor fwr;

//Vision Sensor
extern pros::Vision v_front;

//Color Sensors
extern pros::Optical o_rollerR;
extern pros::Optical o_rollerL;

extern pros::Optical o_flash;

//Distance Sensors
extern pros::Distance d_backL;
extern pros::Distance d_backR;

extern pros::Distance d_frontL;
extern pros::Distance d_frontR;

extern pros::Distance d_discs;

//Radio at port 15
//--------------------------------------------------------------------//
//-----------------------------CONTROLLER-----------------------------//
//--------------------------------------------------------------------//

//-----------Controller
extern Controller controller;

// Controller Buttons
extern ControllerButton btn_fw_shoot;

extern ControllerButton btn_fw_up;
extern ControllerButton btn_fw_down;

extern ControllerButton btn_indexer_spin;

extern ControllerButton btn_intake_toggle;
extern ControllerButton btn_intake_toggleout;
extern ControllerButton btn_intake_in;
extern ControllerButton btn_intake_out;

extern ControllerButton btn_end_on;
extern ControllerButton btn_end_on2;

//--------------------------------------------------------------------//
//-----------------------------ODOM-CONSTANTS-------------------------//
//--------------------------------------------------------------------//

const QLength TRACK_WHEEL_DIAMETER = 2.5_in;
const QLength TRACK_WHEEL_CIRCUMFERENCE = TRACK_WHEEL_DIAMETER * M_PI;
const QLength TRACK_SIDE_OFFSET = 1.5_in;



//--------------------------------------------------------------------//
//-----------------------DRIVETRAIN-CONSTANTS-------------------------//
//--------------------------------------------------------------------//
const QLength DRIVE_WHEEL_DIAMETER = 4.01_in;
const extern double DRIVE_STRAIGHT_SCALE; // converts motor degrees to meters
const extern double DRIVE_TURN_SCALE;     // converts motor degrees to robot degrees
const double DRIVE_WHEEL_DIAMETER_IN = DRIVE_WHEEL_DIAMETER.convert(inch);
const QLength DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * M_PI;
const double DRIVE_WHEEL_CIRCUMFERENCE_IN = DRIVE_WHEEL_CIRCUMFERENCE.convert(inch);
const QLength DRIVE_OFFSET = 14_in;
const double DRIVE_RATIO = 36.0 / 84.0;
const double DRIVE_DEG_TO_IN = DRIVE_RATIO * DRIVE_WHEEL_CIRCUMFERENCE_IN / 360; // * 3 for 600rpm motors

const extern QSpeed DRIVE_MAX_SPEED;        // a measured linear velocity
const extern QAcceleration DRIVE_MAX_ACCEL; // approxamate measured linear acceleration

    /* ------------------------- Settings ------------------------ */
const double SET_DT_MAX = 1;                          // default drivetrain max speed (1 is 100%)
const OdomState SET_ODOM_START = {0_in, 0_in, 0_deg}; // starting position of the robot on the field

const double SET_DT_POW_STRAIGHT = 2; // powers of the curves of the drivetrain control adjustments
const double SET_DT_POW_TURN = 3;

}