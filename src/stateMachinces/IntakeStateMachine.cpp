#include "definitions.hpp"
#include "main.h"
#include "stateMachines/IntakeStateMachine.hpp"

INTAKE_STATES IntakeStateMachine::getState() { return mstate; }
void IntakeStateMachine::setState(const INTAKE_STATES istate)
{
    mstate = istate;
}

void IntakeStateMachine::enableControl()
{
    mcontrolEnabled = true;
}

void IntakeStateMachine::disableControl()
{
    mcontrolEnabled = false;
}

void IntakeStateMachine::controlState() // update the state based on controller input
{

    if (mbtnToggle.changedToPressed()) // if the toggle button is pressed
    {
        if (getState() == INTAKE_STATES::in) // if it's already going in,
            setState(INTAKE_STATES::off);    // make it go out.
        else                                 // otherwise,
            setState(INTAKE_STATES::in);     // make in go in.
        mtoggling = true;                    // note that the system should remain the way it was toggled
    }
    if (mbtnToggleout.changedToPressed()) // if the toggle button is pressed
    {
        if (getState() == INTAKE_STATES::out) // if it's already going in,
            setState(INTAKE_STATES::off);    // make it go out.
        else                                 // otherwise,
            setState(INTAKE_STATES::out);     // make in go in.
        mtoggling = true;                    // note that the system should remain the way it was toggled
    }
    if (mbtnIn.isPressed())
    {
        setState(INTAKE_STATES::in);
        mtoggling = false; // ignore the toggled state in future loops
    }
    else if (mbtnOut.isPressed())
    {
        setState(INTAKE_STATES::out);
        mtoggling = false; // ignore the toggled state in future loops
    }
    else if (mtoggling == false && mstate != INTAKE_STATES::override) // if it's supposed to ignore toggling...
    {
        setState(INTAKE_STATES::off); // just turn off
    }
}

void IntakeStateMachine::update() // move the robot based on the state
{
    switch (mstate)
    {
    case INTAKE_STATES::off:
        mmtr.moveVoltage(0);
        break;
    case INTAKE_STATES::in:
        mmtr.moveVoltage(12000);
        pros::delay(50);
        while (mmtr.getActualVelocity() < 2)
        {
            mmtr.moveVoltage(-12000);
            pros::delay(100);
            mmtr.moveVoltage(12000);
            pros::delay(100);
        }
        break;
    case INTAKE_STATES::out:
        mmtr.moveVoltage(-12000);
        pros::delay(50);
        while (mmtr.getActualVelocity() > -2)
        {
            mmtr.moveVoltage(12000);
            pros::delay(100);
            mmtr.moveVoltage(-12000);
            pros::delay(100);
        }
        break;
    case INTAKE_STATES::override:
        mmtr.moveVoltage(0);
        break;
    case INTAKE_STATES::autonHold2:
        while (def::d_discs.get() > 20) {
            setState(INTAKE_STATES::in);
        } 
        setState(INTAKE_STATES::off);
        break;
    }
}

void IntakeStateMachine::run()
{
    while (true)
    {
        if (mcontrolEnabled)
            controlState();
        update();
        pros::delay(20);
    }
}

/* ----------------------------------------------------------- */
/*                     Private Information                     */
/* ----------------------------------------------------------- */
/* ------------------------- Devices ------------------------- */
Motor &IntakeStateMachine::mmtr = def::mtr_it_m;

/* -------------------------- State -------------------------- */
INTAKE_STATES IntakeStateMachine::mstate = INTAKE_STATES::off;
bool IntakeStateMachine::mtoggling = false;
bool IntakeStateMachine::mcontrolEnabled = false;
INTAKE_STATES IntakeStateMachine::moverrideState = INTAKE_STATES::off;

/* ------------------------- Controls ------------------------ */
ControllerButton &IntakeStateMachine::mbtnToggle = def::btn_intake_toggle;
ControllerButton &IntakeStateMachine::mbtnToggleout = def::btn_intake_toggleout;
ControllerButton &IntakeStateMachine::mbtnIn = def::btn_intake_in;
ControllerButton &IntakeStateMachine::mbtnOut = def::btn_intake_out;