#include "definitions.hpp"
#include "main.h" // gives access to HolderStateMachine.hpp and other dependancies
#include "pros/misc.h"
#include "stateMachines/EndGameStateMachine.hpp"


END_STATES EndGameStateMachine::getState() { return mstate; }
void EndGameStateMachine::setState(const END_STATES istate)
{
    mstate = istate;
}

void EndGameStateMachine::enableControl()
{
    mcontrolEnabled = true;
}

void EndGameStateMachine::disableControl()
{
    mcontrolEnabled = false;
}

void EndGameStateMachine::controlState()
{
    if (mbtnon.changedToPressed() && mbtnon2.changedToPressed()) // if the toggle button is pressed
    {
            def::controller.rumble("- . -.....");
            setState(END_STATES::on);   // close it.
                                           
    }
    else{
            setState(END_STATES::off);     // open it.
    }
}

void EndGameStateMachine::update()
{
    switch (mstate)
    {
    case END_STATES::off:
        if (msol.isEngaged())
            msol.toggle();
        break;
    case END_STATES::on:
        if (!msol.isEngaged()){
            msol.toggle();
        }
        break;
    }
}

void EndGameStateMachine::run()
{
    while (true)
    {
        if (mcontrolEnabled)
            controlState();
        update();
        pros::delay(20);
    }
}

SolenoidWrapper EndGameStateMachine::msol = SolenoidWrapper(def::sol_endg, false);

/* -------------------------- State -------------------------- */
END_STATES EndGameStateMachine::mstate = END_STATES::on;
bool EndGameStateMachine::mcontrolEnabled = false;

/* ------------------------- Controls ------------------------ */
ControllerButton &EndGameStateMachine::mbtnon = def::btn_end_on;  
ControllerButton &EndGameStateMachine::mbtnon2 = def::btn_end_on2;