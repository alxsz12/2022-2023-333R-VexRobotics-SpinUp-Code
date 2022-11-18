/**
 * DrivetrainStateMachine.hpp
 *
 * This file contains the declaration of the DrivetrainStateMachine class.
 * DrivetrainStateMachine is a state machine that inherits from VStateMachine.
 * It has an enumeration of different possible states to make it easy for
 * the user to controll the drivetrain.
 *
 * To use the state machine in auton, you use doAutonMotion() to disable
 * the normal state machine tasks and run the specified action.
 */
#pragma once // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files
class DrivetrainStateMachine // state machine to represent the drivetrain
{
public:
  enum class MStates        // enumeration to organize possible states
  {
    off,
    busy,   // doing an AutonMotion
    manual, // standard split arcade drive
  };
  static MStates getState();
  static void setState(MStates istate);

  static void enableControl();
  static void disableControl();

  static void
  doAutonMotion(std::function<void()> iaction); // disable manual control, and execute the action

  static void controlState(); // update the state based on controller input
  static void update();       // move the robot based on the state
  static void run();          // control the state and update the robot to be run in a separate task

private:
  /* -------------------------- State -------------------------- */
  static MStates mstate, mlastState;
  static bool mcontrolEnabled;

  static bool stateChanged(); // returns whether the last state is the same as the current one

  /* ------------------------- Controls ------------------------ */
  static Controller &mcontroller; // reference to the controller to get joystick values
};

