#include "definitions.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include <chrono>

int count = 0;
INDEXER_STATES IndexerStateMachine::getState() { return mstate; }
void IndexerStateMachine::setState(const INDEXER_STATES istate)
{
    mstate = istate;
}

void IndexerStateMachine::enableControl()
{
    mcontrolEnabled = true;
}

void IndexerStateMachine::disableControl()
{
    mcontrolEnabled = false;
}

void IndexerStateMachine::controlState() // update the state based on controller input
{
    if (mbtnSpin.changedToPressed()) // if the toggle button is pressed
    {
        if (getState() == INDEXER_STATES::off) {// if it's already going in,
            setState(INDEXER_STATES::spin);  
            count++;  // make it go out.
    }else                                 // otherwise,
            setState(INDEXER_STATES::off);     // make in go in.
        mtoggling = true;                    // note that the system should remain the way it was toggled
    }
}

void IndexerStateMachine::update() // move the robot based on the state
{
    switch (mstate)
    {
    case INDEXER_STATES::off:
        mmtr.moveAbsolute(0, 100);
        break;
    case INDEXER_STATES::spin:
     //  while (count == 1)
       //{
            mmtr.moveAbsolute(75, 100);
            pros::delay(100);
            mmtr.moveAbsolute(0, 100);
            pros::delay(100);
            setState(INDEXER_STATES::off);
            count--;
        //}
        
        break;
    }
}

void IndexerStateMachine::run()
{
    while  (true)
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
Motor &IndexerStateMachine::mmtr = def::mtr_ix;

/* -------------------------- State -------------------------- */
INDEXER_STATES IndexerStateMachine::mstate = INDEXER_STATES::off;
bool IndexerStateMachine::mtoggling = false;
bool IndexerStateMachine::mcontrolEnabled = false;

/* ------------------------- Controls ------------------------ */
ControllerButton &IndexerStateMachine::mbtnSpin = def::btn_indexer_spin;
