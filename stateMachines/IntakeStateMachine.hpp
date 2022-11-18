#pragma once
#include "main.h"

class IntakeStateMachine
{
    public:
     enum class MStates // enumeration to organize possible states
    {
        off,      // not doing anything
        in,       // intake rings
        out,      
        override,
        autonHold2,
    };
     static MStates getState();
    static void setState(const MStates istate);

    static void enableControl();
    static void disableControl();

    static void controlState(); // update the state based on controller input
    static void update();       // move the robot based on the state
    static void run();          // control the state and update the robot to be run in separate task

private:
    /* ------------------------- Devices ------------------------- */
    static Motor &mmtr;
    /* -------------------------- State -------------------------- */
    static MStates mstate;
    static bool mtoggling;
    static bool mcontrolEnabled;
    static MStates moverrideState;

    /* ------------------------- Controls ------------------------ */
    static ControllerButton &mbtnToggle; // botton to toggle in/out
    static ControllerButton &mbtnToggleout; // botton to toggle in/out
    static ControllerButton &mbtnIn;     // button to turn the intakes on
    static ControllerButton &mbtnOut;    // button to turn the intakes out
};