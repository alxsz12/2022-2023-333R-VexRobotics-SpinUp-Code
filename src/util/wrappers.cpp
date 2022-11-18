/**
 * wrappers.cpp
 *
 * This file contains the code for wrapping the optical and distance sensors to make organization
 * better.
 */
#include "main.h" // gives access to SensorWrappers and other dependencies

/* ----------------------------------------------------------- */
/*                    DistanceSensorWrapper                    */
/* ----------------------------------------------------------- */
DistanceSensorWrapper::DistanceSensorWrapper(pros::Distance &isensor) : msensor(isensor) {}

int DistanceSensorWrapper::getDistance() { return msensor.get(); }
double DistanceSensorWrapper::getVelocity() { return msensor.get_object_velocity(); }
int DistanceSensorWrapper::getConfidence() { return msensor.get_confidence(); }
int DistanceSensorWrapper::getSize() { return msensor.get_object_size(); }

/* ----------------------------------------------------------- */
/*                     OpticalSensorWrapper                    */
/* ----------------------------------------------------------- */
OpticalSensorWrapper::OpticalSensorWrapper(pros::Optical &isensor) : msensor(isensor) {}

double OpticalSensorWrapper::getHue() { return msensor.get_hue(); }
double OpticalSensorWrapper::getSaturation() { return msensor.get_saturation(); }
double OpticalSensorWrapper::getBrightness() { return msensor.get_brightness(); }
int OpticalSensorWrapper::getProximity() { return msensor.get_proximity(); }
void OpticalSensorWrapper::setLED(int ivalue) { msensor.set_led_pwm(ivalue); }
int OpticalSensorWrapper::getPWM() { return msensor.get_led_pwm(); }
pros::c::optical_direction_e_t OpticalSensorWrapper::getGesture() { return msensor.get_gesture(); }
void OpticalSensorWrapper::enableGestures() { msensor.enable_gesture(); }
void OpticalSensorWrapper::disableGestures() { msensor.disable_gesture(); }

/* ----------------------------------------------------------- */
/*                       SolenoidWrapper                       */
/* ----------------------------------------------------------- */
SolenoidWrapper::SolenoidWrapper(pros::ADIDigitalOut &isolenoid, bool iisEngaged)
    : msolenoid(isolenoid), misEngaged(iisEngaged)
{
    msolenoid.set_value(misEngaged);
} // constructor to set defaults

bool SolenoidWrapper::toggle()
{
    misEngaged = !misEngaged;
    msolenoid.set_value(misEngaged);
    return misEngaged;
}

bool SolenoidWrapper::toggle(const bool iengaged)
{
    misEngaged = iengaged;
    msolenoid.set_value(misEngaged);
    return misEngaged;
}

bool SolenoidWrapper::isEngaged()
{
    return misEngaged;
}