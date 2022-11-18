#include "main.h"

pros::Task sm_dt_task(DrivetrainStateMachine::run);
pros::Task sm_fw_task(FlywheelStateMachine::run);
pros::Task sm_intake_task(IntakeStateMachine::run);
pros::Task sm_indexer_task(IndexerStateMachine::run);
pros::Task sm_endgame_task(EndGameStateMachine::run);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    EndGameStateMachine::setState(END_STATES::off);
    def::imu_top.reset();
    def::imu_side.reset();


    def::mtr_fw_fw1.setGearing(okapi::AbstractMotor::gearset::one);
    def::mtr_fw_fw2.setGearing(okapi::AbstractMotor::gearset::one);

    def::o_rollerL.set_led_pwm(100);
    def::o_rollerR.set_led_pwm(100);

    def::o_flash.set_led_pwm(25);

	def::mtr_dt_left_front.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_right_front.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_right_back.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    def::mtr_dt_left_back.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
        EndGameStateMachine::setState(END_STATES::off);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    EndGameStateMachine::setState(END_STATES::off);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

    waitForImu();
    CustomOdometry::setStateInitial({0_in, 0_in, -def::imu_side.get_rotation() * degree});

    DrivetrainStateMachine::disableControl();
    FlywheelStateMachine::disableControl();
    IntakeStateMachine::disableControl();
    IndexerStateMachine::disableControl();
    EndGameStateMachine::disableControl();

    DrivetrainStateMachine::setState(DT_STATES::busy);




    Drivetrain::straightForDistance(6_in); //Drive Toward the roller
    pros::delay(1000);
        IntakeStateMachine::setState(INTAKE_STATES::in); // Starts the intake for the roller
    pros::delay(1000);
        IntakeStateMachine::setState(INTAKE_STATES::off); // Starts the intake for the next set of discs
    Drivetrain::straightForDistance(-6_in); //Drive away from the roller
    pros::delay(500);
        FlywheelStateMachine::setState(FW_STATES::a2); // Shoots the two preloads
    pros::delay(3000);           //Space out the tasks so they aren't running into each other
    FlywheelStateMachine::setState(FW_STATES::off);
    //Drivetrain::moveArcade(0, -5, false);
    //pros::delay(500);
    //Drivetrain::moveArcade(0, 0, false);
    Drivetrain::turnToAngle(45_deg); // Turns toward the other side of the field
        IntakeStateMachine::setState(INTAKE_STATES::autonHold2); // Starts the intake for the next set of discs
    Drivetrain::straightForDistance(60_in); // Drives forward picking up discs along the way
    Drivetrain::turnToAngle(45_deg); // Turns toward the opposing goal 
        FlywheelStateMachine::setState(FW_STATES::a2); // Shoots the next two discs at the goal
    Drivetrain::turnToAngle(45_deg); // Turns toward the roller 
    Drivetrain::straightForDistance(60_in); //Drives toward the next roller
        IntakeStateMachine::setState(INTAKE_STATES::autonHold2); //Picks up the next discs along the way
    Drivetrain::turnToAngle(45_deg); // Turns toward the roller 
    Drivetrain::straightForDistance(1_ft); //Drives toward the next roller
        IntakeStateMachine::setState(INTAKE_STATES::in); // Starts the intake for the roller
    pros::delay(1000);
        IntakeStateMachine::setState(INTAKE_STATES::off); //Stops when it spins
    Drivetrain::straightForDistance(1_ft); //Drives forward to shoot again      
        FlywheelStateMachine::setState(FW_STATES::a2);
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    sm_dt_task.resume();

    DrivetrainStateMachine::setState(DT_STATES::manual);
    DrivetrainStateMachine::enableControl();

    FlywheelStateMachine::enableControl();
    IntakeStateMachine::enableControl();
    IndexerStateMachine::enableControl();
    EndGameStateMachine::enableControl();
}
