#pragma once
#include "main.h"
#include "util/wrappers.hpp"

class EndGameStateMachine
{
public:
    enum class MStates
    {
        off, 
        on,
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
    static SolenoidWrapper msol;
    /* -------------------------- State -------------------------- */
    static MStates mstate;
    static bool mtoggling;
    static bool mcontrolEnabled;

    /* ------------------------- Controls ------------------------ */
    static ControllerButton &mbtnon; 
    static ControllerButton &mbtnon2; 
};