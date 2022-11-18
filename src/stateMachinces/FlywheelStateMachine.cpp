#include "definitions.hpp"
#include "main.h"
#include "okapi/api/util/logging.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "pros/motors.hpp"
#include "stateMachines/FlywheelStateMachine.hpp"

int fwv = 9000;

FW_STATES FlywheelStateMachine::getState() { return mstate; }
void FlywheelStateMachine::setState(const FW_STATES istate)
{
    mstate = istate;
}

void FlywheelStateMachine::enableControl()
{
    mcontrolEnabled = true;
}

void FlywheelStateMachine::disableControl()
{
    mcontrolEnabled = false;
}

void FlywheelStateMachine::controlState(){
    if(mbtnShoot.changedToPressed()){
        if (getState() == FW_STATES::off) // if it's already shooting
            setState(FW_STATES::shoot);    // make it stop.
        else 
            setState(FW_STATES::off);  
    }
}   

void FlywheelStateMachine::speedu(){
    if(mbtnUp.changedToPressed()){
        fwv+= 500;
        def::controller.setText(0, 0, "Flywheel: fwv");
    }
    if(mbtnDown.changedToPressed()){
        fwv-= 500;
        def::controller.setText(0, 0, "Flywheel: fwv");
    }
}

void FlywheelStateMachine::update() // updates the subsystem based on the state
{
    switch (mstate)
    {
        case FW_STATES::off: // Turns the flywheel Off
            mmotors.moveVoltage(0);
            break;
        case FW_STATES::shoot: // Spins the flywheel 
            mmotors.moveVoltage(fwv);    
            break;
        case FW_STATES::a2: // State for autonomous to shoot the two preloads then stop.
            mmotors.moveVoltage(10500); //Starts the flywheel;
                    pros::delay(1000); // Wait one second before starting the indexer
                IndexerStateMachine::setState(INDEXER_STATES::spin); // Fires the first disc
                    pros::delay(500); // Waits half a second before firing another disc
                IndexerStateMachine::setState(INDEXER_STATES::spin); //Fires the second preload
                pros::delay(1000);
            break;
    }
}

void FlywheelStateMachine::run()
{
    while (true)
    {
        if (mcontrolEnabled)
                controlState();
            speedu();
        update();
        pros::delay(20);
    }
}

/* ------------------------- Devices ------------------------- */
MotorGroup FlywheelStateMachine::mmotors = MotorGroup({def::mtr_fw_fw1, def::mtr_fw_fw2});
/* -------------------------- State -------------------------- */

//FW_STATES FlywheelStateMachine::moverrideState = FW_STATES::off;
FW_STATES FlywheelStateMachine::mstate = FW_STATES::off;

bool FlywheelStateMachine::mcontrolEnabled = false;

ControllerButton &FlywheelStateMachine::mbtnShoot = def::btn_fw_shoot;

ControllerButton &FlywheelStateMachine::mbtnUp = def::btn_fw_up;
ControllerButton &FlywheelStateMachine::mbtnDown = def::btn_fw_down;
