#include "definitions.hpp"
#include "main.h"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "stateMachines/FlywheelStateMachine.hpp"


namespace def{
//--------------------------------------------------------------------//
//-----------------------------MOTORS---------------------------------//
//--------------------------------------------------------------------//

//-----------Drivetrain Motors
Motor mtr_dt_left_front(-2); //1
Motor mtr_dt_left_back(-3); //4
Motor mtr_dt_right_front(1); //2
Motor mtr_dt_right_back(4); //3


//-----------Indexer Motor-----
Motor mtr_ix(-5);

//-----------Intake Motors------
Motor mtr_it_m(6);

//Flywheel Motors
Motor mtr_fw_fw1(7);
Motor mtr_fw_fw2(-8);


//Pnumatics
pros::ADIDigitalOut sol_endg('H');
//--------------------------------------------------------------------//
//-----------------------------SENSORS--------------------------------//
//--------------------------------------------------------------------//

//Encoders
ADIEncoder track_encoder_side('A', 'B', true);

//-----------Inertial Sensors
pros::Imu imu_side(9);
pros::Imu imu_top(19);


//Rotaition Sensors
RotationSensor fwr(10, false);

//Vision Sensor
pros::Vision v_front(11);

//Color Sensors
pros::Optical o_rollerR(12);
pros::Optical o_rollerL(13);

pros::Optical o_flash(18);

//Distance Sensors
pros::Distance d_backL(14);
pros::Distance d_backR(16);

pros::Distance d_frontL(17);
pros::Distance d_frontR(21);

pros::Distance d_discs(20);

//Radio at port 15

//--------------------------------------------------------------------//
//-----------------------------CONTROLLER-----------------------------//
//--------------------------------------------------------------------//

//-----------Controller
Controller controller = Controller();

//Controller Button Definitions
ControllerButton btn_indexer_spin = ControllerDigital::L2;

ControllerButton btn_fw_shoot = ControllerDigital::L1;

ControllerButton btn_fw_up = ControllerDigital::up;
ControllerButton btn_fw_down = ControllerDigital::down;


ControllerButton btn_intake_toggle = ControllerDigital::X;
ControllerButton btn_intake_toggleout = ControllerDigital::Y;
ControllerButton btn_intake_in = ControllerDigital::R1;
ControllerButton btn_intake_out = ControllerDigital::R2;

ControllerButton btn_end_on = ControllerDigital::left;
ControllerButton btn_end_on2 = ControllerDigital::right;

// State Machince Definitions
CustomOdometry customOdom = CustomOdometry(); // object that calculates position

Drivetrain drivetrain = Drivetrain(); // used by DrivetrainStateMachine for drivetrain control

DrivetrainStateMachine sm_dt = DrivetrainStateMachine(); // state machine to control the drivetrain

const double DRIVE_STRAIGHT_SCALE = 1922;
const double DRIVE_TURN_SCALE = 17;
const QSpeed DRIVE_MAX_SPEED = 1.3_mps;      // a measured linear velocity
const QAcceleration DRIVE_MAX_ACCEL = 1.1_G; // approxamate measured linear acceleration
}
