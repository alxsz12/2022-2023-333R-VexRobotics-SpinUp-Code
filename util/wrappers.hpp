/**
 * wrappers.hpp
 *
 * This file contains the code for wrapping the optical and distance sensors to make organization
 * better.
 */

#pragma once      // makes sure the file is only included once
#include "main.h" // gives access to dependancies from other files

class DistanceSensorWrapper
{
  pros::Distance &msensor;

public:
  DistanceSensorWrapper(pros::Distance &isensor); // wraps a sensor reference

  int getDistance();    // in mm
  double getVelocity(); // in m/s
  int getConfidence();  // in range [0, 63], 63 being the highest confidence, only available when
                        // distance > 20cm
  int getSize();        // in range [0, 400]
};

class OpticalSensorWrapper
{
  pros::Optical &msensor;

public:
  OpticalSensorWrapper(pros::Optical &isensor); // wraps a sensor reference

  double getHue();         // in range [0, 359.999]
  double getSaturation();  // in range [0, 1]
  double getBrightness();  // in range [0, 1]
  int getProximity();      // in range [0, 255]
  void setLED(int ivalue); // in range [0, 100]
  int getPWM();            // in range [0, 100]
  pros::c::optical_direction_e_t
  getGesture(); // gets the direction of a gesture in the past 500 ms
  void enableGestures();
  void disableGestures();
};

class SolenoidWrapper
{
  pros::ADIDigitalOut &msolenoid;

  bool misEngaged;

public:
  SolenoidWrapper(pros::ADIDigitalOut &isolenoid, bool iisEngaged = false); // constructor to set defaults

  bool toggle(); // switch the status of the solenoid
  bool toggle(const bool iengaged);

  bool isEngaged(); // check the status of the solenoid
};