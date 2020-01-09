#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////
//motor direction
#define LEFT_BASE_REVERSED false
#define RIGHT_BASE_REVERSED true
#define LEFT_BASE_2_REVERSED false
#define RIGHT_BASE_2_REVERSED true

#define LIFT_REVERSED false

#define LEFT_ROLLER_REVERSED false
#define RIGHT_ROLLER_REVERSED true

#define TRAY_REVERSED false

//motor ports
#define LEFT_BASE_PORT 8
#define RIGHT_BASE_PORT 3
#define LEFT_BASE_2_PORT 4
#define RIGHT_BASE_2_PORT 6

#define LIFT_PORT 12

#define LEFT_ROLLER_PORT 18
#define RIGHT_ROLLER_PORT 13

#define TRAY_PORT 19


//sets up controller and motor objects
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_mtr(LEFT_BASE_PORT, pros::E_MOTOR_GEARSET_18, LEFT_BASE_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_mtr(RIGHT_BASE_PORT, pros::E_MOTOR_GEARSET_18, RIGHT_BASE_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_mtr2(LEFT_BASE_2_PORT, pros::E_MOTOR_GEARSET_18, LEFT_BASE_2_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_mtr2(RIGHT_BASE_2_PORT, pros::E_MOTOR_GEARSET_18, RIGHT_BASE_2_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor lift_mtr(LIFT_PORT, pros::E_MOTOR_GEARSET_18, LIFT_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_roller_mtr(LEFT_ROLLER_PORT, pros::E_MOTOR_GEARSET_18, LEFT_ROLLER_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_roller_mtr(RIGHT_ROLLER_PORT, pros::E_MOTOR_GEARSET_18, RIGHT_ROLLER_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor tray_mtr(TRAY_PORT, pros::E_MOTOR_GEARSET_18, TRAY_REVERSED, pros::E_MOTOR_ENCODER_DEGREES);




int trueSpeed[] =   {0,  3,  3,  3,  3,  3, 10, 10, 10, 10,
                    10, 10, 10, 10, 12, 12, 12, 12, 12, 12,
                    14, 14, 14, 14, 14, 14, 20, 20, 20, 20,
                    20, 20, 20, 25, 25, 25, 25, 25, 25, 25,
                    30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
                    45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
                    55, 55, 55, 55, 55, 55, 55, 55, 55, 55,
                    66, 66, 66, 66, 66, 66, 66, 66, 66, 66,
                    79, 79, 79, 79, 79, 79, 79, 79, 79, 79,
                    89, 89, 89, 89, 89, 89, 89, 89, 89, 89,
                    95, 95, 95, 99, 99, 99, 99, 104,104,104,
                    107,107,107,113,113,113,113,113,116,116,
                    116,116,123,123,123,127,127,127};

/*
int trueSpeed[] =  {0,  0,  3,  3,  3,  3,  10, 10, 10, 10,
									  10, 10, 10, 10, 12, 12, 12, 12, 12, 12,
									  14, 14, 14, 14, 14, 14, 20, 20, 20, 20,
									  20, 20, 20, 25, 25, 25, 25, 25, 25, 30,
									  30, 30, 30, 30, 30, 45, 45, 45, 45, 45,
									  45, 45, 45, 55, 55, 55, 55, 66, 66, 66,
									  66, 79, 79, 79, 79, 79, 89, 89, 89, 89,
									  89, 89, 89, 95, 95, 95, 99, 99, 99, 99,
									  104,104,104,107,107,107,113,113,113,113,
									  113,116,116,116,116,123,123,123,127,127,
									  127};
*/

//debugging variables
bool const printBaseVals = false;
bool const printLiftVals = false;
bool const printRollerVals = false;
bool const printTrayVals = false;

//variables for lift PID
static const double liftKp = 1.5;
static double liftSetVal = 0;
static double liftErr = 0;
static double currLiftVal = 0;
static bool liftHold = false;

//variables for left roller PID
static const double leftRollerKp = 1.5;
static double leftRollerSetVal = 0;
static double leftRollerErr = 0;
static double currLeftRollerVal = 0;
//variables for right roller PID
static const double rightRollerKp = 1.5;
static double rightRollerSetVal = 0;
static double rightRollerErr = 0;
static double currRightRollerVal = 0;
//shared roller variables
static bool rollerHold = false;

//variables for tray PID
static const double trayKp = 1.5;
static double traySetVal = 0;
static double trayErr = 0;
static double currTrayVal = 0;
static bool trayHold = false;

//variables for the automated scoring sequence
static const double autoTrayKp = 0.1;
static const double autoTraySetVal = -1715;
static bool autoTray = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////
//pos track variables
bool const printPosTrackVal = true;

double currX = 0.0; //current robot x position
double currY = 0.0; //current robot y position
double currA = 0.0; //current robot heading

double dX = 0.0; //change in x
double dY = 0.0; //change in y
double dA = 0.0; //change in robot heading
double dR = 0.0; //hypotenuse of the triangle with legs being dX adn dY

int currLEnc = 0; //current left base encoder value
int currREnc = 0; //current right base encoder value
int lastLEnc = 0; //current left base encoder value
int lastREnc = 0; //current right base encoder value
int dLEnc = 0;
int dREnc = 0;

double rLCOR = 0; //(radius of Left Center Of Rotation) The distance from the IDEAL left encoder wheel position to the left center of rotation (when making a left turn)
double rRCOR = 0; //(radius of Right Center Of Rotation) The distance from the IDEAL right encoder wheel position to the right center of rotation (when making a right turn)
double rMCOR = 0; //(radius of Middle Center of Rotation) The distance from the IDEAL right encoder wheel position to the center of the rotation (when making a point turn)

const int refreshRate = 500; //in Hz

//encoder offset calculation variables
const double encLRDist = 11.6 * 360 / (4 * M_PI); //distance between the left and right base encoders
const double encCMidDist = 5.2 * 360 / (4 * M_PI); //distance from the center base encoder to its IDEAL position

///////////////////////////////////////////////////////////////////////////////////////////////////////

int returnTrueSpeed (int controllerVal) {
	int trueSpeedVal = 0;
	if (controllerVal >= 0) {
    trueSpeedVal = (int)(100.0/127*(trueSpeed[abs(controllerVal)]));
	}
	else if (controllerVal < 0) {
    trueSpeedVal = (int)(100.0/127*(-trueSpeed[abs(controllerVal)]));
	}
	return trueSpeedVal;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}
/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
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
  //base drive back
  left_mtr = -100;
	left_mtr2 = -100;
	right_mtr = -100;
	right_mtr2 = -100;
  pros::delay(300);
  //base drive forward and tray up
  left_mtr = 100;
	left_mtr2 = 100;
	right_mtr = 100;
	right_mtr2 = 100;
  tray_mtr = -127;
  pros::delay(300);
  //base stop and tray back down
  left_mtr = 0;
	left_mtr2 = 0;
	right_mtr = 0;
	right_mtr2 = 0;
  tray_mtr = 127;
  pros::delay(300);
  /*
  //turn left
  left_mtr = -80;
	left_mtr2 = -80;
	right_mtr = 80;
	right_mtr2 = 80;
  pros::delay(400);
  //stop
  left_mtr = 0;
	left_mtr2 = 0;
	right_mtr = 0;
	right_mtr2 = 0;
  pros::delay(300);
  //go straight
  left_mtr = 100;
	left_mtr2 = 100;
	right_mtr = 100;
	right_mtr2 = 100;
  pros::delay(500);
  */

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
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

///////////////////////////////////////////////////////////////////////////////////////////////////////

	 	//left base true speed
	 	left_mtr = returnTrueSpeed (master.get_analog(ANALOG_LEFT_Y));
		left_mtr2 = returnTrueSpeed (master.get_analog(ANALOG_LEFT_Y));
	 	//right base true speed
	  right_mtr = returnTrueSpeed (master.get_analog(ANALOG_RIGHT_Y));
		right_mtr2 = returnTrueSpeed (master.get_analog(ANALOG_RIGHT_Y));
	 	if (printBaseVals){
	     	pros::lcd::print(3, "left_mtr: %d", left_mtr);
				pros::lcd::print(4, "left_mtr2: %d", left_mtr2);
				pros::lcd::print(5, "right_mtr: %d", right_mtr);
				pros::lcd::print(6, "right_mtr2: %d", right_mtr2);
	 	}

///////////////////////////////////////////////////////////////////////////////////////////////////////


		//lift
		currLiftVal = lift_mtr.get_position();
		liftErr = liftSetVal - currLiftVal;
		//lift_mtr = liftErr * liftKp;
		if (printLiftVals) {
			pros::lcd::print(3, "liftHold: %d", liftHold);
			pros::lcd::print(4, "currLiftVal: %d", currLiftVal);
			pros::lcd::print(5, "liftSetVal: %d", liftSetVal);
			pros::lcd::print(6, "liftErr: %d", liftErr);
			pros::lcd::print(7, "lift_mtr: %d", (liftErr * liftKp));
		}
		//Manual lift code
		if (master.get_digital(DIGITAL_R1)) {
			liftHold = false;
			lift_mtr = -127;
		}
	  else if (master.get_digital(DIGITAL_R2)) {
			liftHold = false;
			lift_mtr = 127;
		}
		else if (liftHold == false) {
			liftHold = true;
			liftSetVal = currLiftVal;
		}
		//lift PID hold
		if (liftHold) {
			lift_mtr = liftErr * liftKp;
		}


///////////////////////////////////////////////////////////////////////////////////////////////////////


		//roller
		currLeftRollerVal = left_roller_mtr.get_position();
		leftRollerErr = leftRollerSetVal - currLeftRollerVal;
		currRightRollerVal = right_roller_mtr.get_position();
		rightRollerErr = rightRollerSetVal - currRightRollerVal;

		if (printRollerVals) {
			pros::lcd::print(3, "rollerHold: %d", rollerHold);
			pros::lcd::print(4, "currLeftRollerVal: %d", currLeftRollerVal);
			pros::lcd::print(5, "leftRollerSetVal: %d", leftRollerSetVal);
			pros::lcd::print(6, "leftRollerErr: %d", leftRollerErr);
			pros::lcd::print(7, "left_roller_mtr: %d", (leftRollerErr * leftRollerKp));
		}
		//Manual roller code
		if (master.get_digital(DIGITAL_L1)) {
			rollerHold = false;
			left_roller_mtr = -127;
			right_roller_mtr = -127;
		}
	  else if (master.get_digital(DIGITAL_L2)) {
			rollerHold = false;
			left_roller_mtr = 127;
			right_roller_mtr = 127;
		}
		else if (rollerHold == false) {
			rollerHold = true;
			leftRollerSetVal = currLeftRollerVal;
			rightRollerSetVal = currRightRollerVal;
		}
		//roller PID hold
		if (rollerHold) {
			left_roller_mtr = leftRollerErr * leftRollerKp;
			right_roller_mtr = rightRollerErr * rightRollerKp;
		}


///////////////////////////////////////////////////////////////////////////////////////////////////////


		//tray
		currTrayVal = tray_mtr.get_position();
		trayErr = traySetVal - currTrayVal;
    int diplayVal = (int) currTrayVal;
		if (printTrayVals) {
			pros::lcd::print(3, "trayHold: %d", trayHold);
			pros::lcd::print(4, "currTrayVal: %d", diplayVal);
			pros::lcd::print(5, "traySetVal: %d", traySetVal);
			pros::lcd::print(6, "trayErr: %d", trayErr);
			pros::lcd::print(7, "tray_mtr: %d", (trayErr * trayKp));
		}
		//Manual tray code
		if (master.get_digital(DIGITAL_A)) {
			trayHold = false;
			tray_mtr = -127;
		}
	  else if (master.get_digital(DIGITAL_B)) {
			trayHold = false;
			tray_mtr = 127;
		}
		else if (master.get_digital(DIGITAL_X)) {
			trayHold = false;
			tray_mtr = -32;
		}
		else if (master.get_digital(DIGITAL_Y)) {
			trayHold = false;
			tray_mtr = 32;
		}
		else if (trayHold == false) {
			trayHold = true;
			traySetVal = currTrayVal;
		}
		//tray PID hold
		if (trayHold) {
			tray_mtr = trayErr * trayKp;
		}


///////////////////////////////////////////////////////////////////////////////////////////////////////


    //stacking macro
    if (master.get_digital(DIGITAL_UP)) {
      liftHold = false;
      //rollerHold = false;
      trayHold = false;
      autoTray = true;
      traySetVal = autoTraySetVal;
    }
    //if autoTray is activated, and non of the coded buttons besides base joysticks are pressed
    if (autoTray && !(master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_L2) || master.get_digital(DIGITAL_A) || master.get_digital(DIGITAL_B) || master.get_digital(DIGITAL_X) || master.get_digital(DIGITAL_Y))) {
      tray_mtr = trayErr * autoTrayKp;
    }
    //if any of the coded buttons are pressed, it cancels the autoTray mode
    else if (master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_L2) || master.get_digital(DIGITAL_A) || master.get_digital(DIGITAL_B) || master.get_digital(DIGITAL_X) || master.get_digital(DIGITAL_Y)) {
      autoTray = false;
    }


///////////////////////////////////////////////////////////////////////////////////////////////////////


    //updating position
		currA += dA;
    dX = dR *cos(currA); //argument in radians
    dY = dR *sin(currA); //argument in radians
    currX += dX;
    currY += dY;
    if (printPosTrackVal) {
      pros::lcd::print(3, "X: %f, Y: %f", currX, currY);
		  pros::lcd::print(4, "A(degrees): %f", (currA * (180/M_PI)));
		  pros::lcd::print(5, "LEnc: %d, REnc: %d", currLEnc, currREnc);
    }

		currLEnc = (int) left_mtr.get_position();
		currREnc = (int) right_mtr.get_position();

		/*
		currLEnc = right_mtr2.get_position();
		currREnc = left_mtr.get_position();
		*/

		dR = (dLEnc + dREnc) / 2;
		dA = (dREnc - dLEnc) / encLRDist;

		dLEnc = currLEnc - lastLEnc;
		dREnc = currREnc - lastREnc;

		lastLEnc = currLEnc;
		lastREnc = currREnc;


///////////////////////////////////////////////////////////////////////////////////////////////////////

		pros::delay(1000/refreshRate);
	}
}
