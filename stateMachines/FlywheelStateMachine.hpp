#pragma once
#include "main.h"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "stateMachines/VStateMachine.hpp"

class FlywheelStateMachine 
{

public:
    enum class MStates        // enumeration to organize possible states
    {
      off,
      shoot, // standard split arcade drive
      a2,
    };
    static MStates getState();
    static void setState(MStates istate);

    static void enableControl();
    static void disableControl();

    static void controlState(); // update the state based on controller input

    static void speedu();

    static void update();       // move the flywheel based on the state
    static void run();    

    
private:
    static MotorGroup mmotors;

    static MStates mstate;
    static bool mcontrolEnabled;
    
    static ControllerButton &mbtnShoot;   
    static ControllerButton &mbtnUp;    
    static ControllerButton &mbtnDown;    
};
