#pragma once
#include "main.h"

class IndexerStateMachine // state machine to represent the drivetrain
{
public:
    enum class MStates // enumeration to organize possible states
    {
        off,      
        spin,       
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

    /* ------------------------- Controls ------------------------ */
    static ControllerButton &mbtnSpin;     // button to turn the intakes on
};