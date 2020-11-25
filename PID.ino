/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
 * using a basic line following algorithm. This example works on a dark floor with
 * a white line or a light floor with a dark line. The robot first needs to be calibrated
 * Then place the robot on the hit the left button again to begin the line following.
 *
 * How to run:
 * 1) Push left button on Launchpad to have the robot perform calibration.
 * 2) Robot will drive forwards and backwards by a predefined distance.
 * 3) Place the robot center on the line you want it to follow.
 * 4) Push left button again to have the robot begin to follow the line.
 *
 * Parts Info:
 * o Black eletrical tape or white electrical tape. Masking tape does not work well
 *   with IR sensors.
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

// default, max, and min speed of the wheels
// the correction speed is used if the encoder
// values are out of sync and a wheel needs to sped up
#define BASE_SPEED 25
#define CORRECTION_SPEED_DELTA 5 // 5%
#define BUFF_SIZE 7

/// PID Const Variables
#define PID_OUT_MIN -49
#define PID_OUT_MAX 49
#define PID_Kp 0.022 // 0.022
#define PID_Ki 0.000025 // 0.0001
#define PID_Kd 0.003 // 0.003
#define PID_SETPOINT 3500

/// Binds a variable to a lower or upper bind
/// I thought it would be used more often, I guess not
#define LOWER_BIND(X, Y) if ((X) < (Y)) { (X) = (Y); }
#define UPPER_BIND(X, Y) if ((X) > (Y)) { (X) = (Y); }

#define LINE_LOST_COUNTER_MAX 1'000'000

// Various constants defining the physical characteristics
// of the vehicle
#define WHEEL_DIAM 0.06959 // in meters
#define WHEEL_BASE 0.143 // in meters
#define CNT_PER_REV 360 // Number of encoder (rising) pulses every time the wheel turns completely


// Percent of journey to stop acceleration
// and start deceleration
#define SPEED_STEP_ACCEL 0.25 // 0% to 25% accelerate
#define SPEED_STEP_DECEL 0.75 // 75% to 100% decelerate

/// Converts the an angle of rotation to the amount the
/// wheels will travel. Assumes vehicle will use both wheels
/// to rotate
///
/// @param degrees The degrees of rotation to convert
/// @param WHEEL_BASE The distance between the 2 wheels
///        Should be renamed to WHEEL_TRACK
/// @return The distance one wheel travels in meters
float calc_degrees_to_distance_spin (float degrees) {
	float wheel_base_circum = WHEEL_BASE * PI;
	float angle_ratio = degrees / 360.0;
	return angle_ratio * wheel_base_circum;
}

/// Converts wheel travel distance to an encoder count
/// Useful for calculateing how many encoder counts needed
/// to travel a distance
///
/// @param distance The distance to convert
/// @param WHEEL_DIAM The diameter of the wheel
/// @param CNT_PER_REV The number of encoder counts per
///        one revolution of the wheel
/// @return The number of encoder counts it should take
///         to travel \param distance distance
uint16_t calc_enoder_count (float distance) {
	float wheel_circum = WHEEL_DIAM * PI;
	float wheel_rotations = distance / wheel_circum;
	return uint16_t( wheel_rotations * CNT_PER_REV );
}

/// Keeps the motor in sync using the encoders until both
/// encoders have reached \param cnt count. Also will
/// run a speed control function to adjust the speed as it
/// travels
///
/// @param cnt The number of encoder counts to keep sync until
/// @param speed_ctrl_func A functional pointer used to determine the
///        base speed of the motors, used to transform the speed thoughout
///        the journey of the vehicle
/// @param BASE_SPEED The default wheel speed when no transform is used
void sync_motors_until_cnt(uint16_t cnt, uint8_t (*speed_ctrl_func)(float)) {
	uint16_t leftCount = getEncoderLeftCnt();
	uint16_t rightCount = getEncoderRightCnt();
	uint8_t baseSpeed = BASE_SPEED;
	uint8_t leftMotorSpeed = baseSpeed;
	uint8_t rightMotorSpeed = baseSpeed;

	while (leftCount < cnt and rightCount < cnt) {
		uint16_t avgCount = (leftCount + rightCount) / 2;
		float progress = float(avgCount) / cnt;

		// first get base speed using the transform function
		if (speed_ctrl_func != nullptr)
			baseSpeed = speed_ctrl_func(progress);
		else
			baseSpeed = BASE_SPEED;

		Serial.print(baseSpeed);
		Serial.print("\t");

		leftCount = getEncoderLeftCnt();
		rightCount = getEncoderRightCnt();

		Serial.print(leftCount);
		Serial.print("\t");
		Serial.print(rightCount);
		Serial.print("\t");

		// then adjust the left and right wheel speeds to sync up
		// the wheel. only gets synced up if the encoder counts there
		// is more than 10 counts of difference otherwise reset the
		// wheel speeds back to default
		int abs_diff = abs(int(leftCount) - rightCount);
		if (abs_diff > 10) {
			Serial.print("off_");

			if (leftCount < rightCount) { // left wheel lagging
				leftMotorSpeed = baseSpeed + CORRECTION_SPEED_DELTA;
				rightMotorSpeed = baseSpeed - CORRECTION_SPEED_DELTA;

				Serial.print("inc_left");
			}

			if (rightCount < leftCount) { // right wheel lagging
				leftMotorSpeed = baseSpeed - CORRECTION_SPEED_DELTA;
				rightMotorSpeed = baseSpeed + CORRECTION_SPEED_DELTA;

				Serial.print("inc_right");
			}

			// Check bounds for under/overflow
			// if (leftMotorSpeed <= speed_inc) {
			// 	leftMotorSpeed = speed_inc;
			// }

			// if (rightMotorSpeed <= speed_inc) {
			// 	rightMotorSpeed = speed_inc;
			// }

			// if (leftMotorSpeed >= 100) {
			// 	leftMotorSpeed = 100;
			// }

			// if (rightMotorSpeed >= 100) {
			// 	rightMotorSpeed = 100;
			// }

			Serial.print("\t");
			Serial.print(leftMotorSpeed);
			Serial.print("\t");
			Serial.print(rightMotorSpeed);
		} else {
			leftMotorSpeed = baseSpeed;
			rightMotorSpeed = baseSpeed;

			Serial.print("in_sync");
		}

		setMotorSpeed(LEFT_MOTOR,leftMotorSpeed);
		setMotorSpeed(RIGHT_MOTOR,rightMotorSpeed);

		Serial.println();
	}
}

/// Spins the vehicele using both wheels \param degrees number
/// of degrees.
///
/// @param degrees The number of degrees to spin the vehicle, positive
///        numbers mean CCW, negative numbers mean CW
void spin(float degrees) {
	float distance_to_turn = calc_degrees_to_distance_spin(abs(degrees));
	uint16_t encoder_cnt_needed = calc_enoder_count(distance_to_turn);

	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	if (degrees > 0) {
		setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
		setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
	} else {
		setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
		setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
	}

	enableMotor(BOTH_MOTORS);
	setMotorSpeed(BOTH_MOTORS, BASE_SPEED);

	sync_motors_until_cnt(encoder_cnt_needed, nullptr);

	disableMotor(BOTH_MOTORS);
}

/// Update the PID values for the current system
/// and return the new output. If a value of
/// (-1, -1, -1) is passed in as parameters, it resets
/// the internal values to 0. This is useful if he vehicle
/// is moved and there are bad values in the integration
/// history.
///
/// @ref https://gist.github.com/bradley219/5373998
///
/// @param dt change in time from last func call
/// @param cur current PID input value
/// @param setpoint value PID input should be
int8_t update_pid(float dt, float cur, float setpoint = PID_SETPOINT) {
	static float integral = 0;
	static float pre_error = 0;

	if (dt == -1 and cur == -1 and setpoint == -1) {
		integral = 0;
		pre_error = 0;
	}

	// Calculate error
	float error = setpoint - cur;

	// Proportional term
	float Pout = PID_Kp * error;

	// Integral term
	integral += error * dt; // millis -> seconds
	float Iout = PID_Ki * integral;
	// Serial.print(" DT: ");
	// Serial.print(dt);
	// Serial.print(" ");
	// Serial.print(" ER: ");
	// Serial.print(error);
	// Serial.print(" ");
	// Serial.print(" IG: ");
	// Serial.print(integral);
	// Serial.print(" ");

	// Derivative term
	float ddt = (error - pre_error) / dt;
	float Dout = PID_Kd * ddt;

	// Calculate total output
	float output = Pout + Iout + Dout;

	// Restrict to max/min
	UPPER_BIND(output, PID_OUT_MAX);
	LOWER_BIND(output, PID_OUT_MIN);

	// Save error to previous error
	pre_error = error;

	return output;
}

bool isCalibrationComplete = true;
void setup()
{
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
	clearMinMax(sensorMinVal,sensorMaxVal);

	// Some default values so I dont have to calibrate
	// it every time during PID tuning
	sensorMinVal[0] = 830;
	sensorMinVal[1] = 904;
	sensorMinVal[2] = 643;
	sensorMinVal[3] = 721;
	sensorMinVal[4] = 656;
	sensorMinVal[5] = 804;
	sensorMinVal[6] = 751;
	sensorMinVal[7] = 873;

	sensorMaxVal[0] = 944;
	sensorMaxVal[1] = 1033;
	sensorMaxVal[2] = 733;
	sensorMaxVal[3] = 829;
	sensorMaxVal[4] = 732;
	sensorMaxVal[5] = 908;
	sensorMaxVal[6] = 860;
	sensorMaxVal[7] = 975;

	/* Run this setup only once */
	if(isCalibrationComplete == false) {
		floorCalibration();
		isCalibrationComplete = true;
	} else

	Serial.println("Finished with setup");
}

void floorCalibration() {
	/* Place Robot On Floor (no line) */
	delay(2000);
	String btnMsg = "Push left button on Launchpad to begin calibration.\n";
	btnMsg += "Make sure the robot is on the floor away from the line.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

	delay(1000);

	Serial.println("Running calibration on floor");
	simpleCalibrate();
	Serial.println("Reading floor values complete");

	btnMsg = "Push left button on Launchpad to begin line following.\n";
	btnMsg += "Make sure the robot is on the line.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
	delay(1000);

	enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
	/* Set both motors direction forward */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
	/* Enable both motors */
	enableMotor(BOTH_MOTORS);
	/* Set both motors speed 20 */
	setMotorSpeed(BOTH_MOTORS,20);

	for(int x = 0;x<100;x++){
		readLineSensor(sensorVal);
		setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
	}

	/* Disable both motors */
	disableMotor(BOTH_MOTORS);
}

void clear_buff(uint32_t* buffer, size_t size) {
	while (size --> 0) {
		buffer[size] = 0;
	}
}

size_t add_buff(uint32_t* buffer, size_t size, size_t idx, uint32_t val) {
	buffer[idx++] = val;
	return idx % size;
}

uint32_t avg_buff(uint32_t* buffer, size_t size) {
	uint32_t sum = 0;
	for (int i = 0; i < size; ++i) {
		sum += buffer[i];
	}
	return round(float(sum) / size);
}

void loop() {
	static uint64_t line_lost_counter = 0;
	static unsigned long last_time = 0;
	static bool done = true;
	static uint32_t last_line_pos[BUFF_SIZE];
	static size_t buff_cur = 0;

	// Wait for button press after a run so we can run again
	if (done) {
		disableMotor(BOTH_MOTORS);
		waitBtnPressed(LP_LEFT_BTN,"Waiting for Button Press",RED_LED);
		delay(2000);
		clear_buff(last_line_pos, BUFF_SIZE);
		enableMotor(BOTH_MOTORS);
		update_pid(-1, -1, -1);
		done = false;
	}

	// read sensor values
	readLineSensor(sensorVal);
	readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,DARK_LINE);

	// Get line position
	uint32_t val = getLinePosition(sensorCalVal,DARK_LINE);
	buff_cur = add_buff(last_line_pos, BUFF_SIZE, buff_cur, val);
	uint32_t linePos = avg_buff(last_line_pos, BUFF_SIZE);
    // Serial.print(linePos);

	int8_t leftMotorSpeed = BASE_SPEED, rightMotorSpeed = BASE_SPEED, change = 0;

	// if the position is 0 then no line is under the vehicle
	// and line is lost
	if (linePos == 0) {
		Serial.println("Line Lost");
		++line_lost_counter;

		leftMotorSpeed  = (BASE_SPEED - change);
		rightMotorSpeed = (BASE_SPEED + change);

		if (line_lost_counter > LINE_LOST_COUNTER_MAX) {
			done = true;
			return;
		}
	} else {
		if (line_lost_counter > 0) {
			line_lost_counter = 0;
			update_pid(-1, -1, -1);
		}

		uint32_t mid2SensorVal = (sensorCalVal[3] + sensorCalVal[4]) / 2;
		Serial.print(mid2SensorVal);
		if (mid2SensorVal > 400) {
			uint32_t left2SensorVal = (sensorCalVal[6] + sensorCalVal[7]) / 2;
			uint32_t right2SensorVal = (sensorCalVal[0] + sensorCalVal[1]) / 2;
			if (left2SensorVal > 500 and right2SensorVal < 500) {
				spin(-90);
				enableMotor(BOTH_MOTORS);
				setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
				setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
			}

			if (right2SensorVal > 500 and left2SensorVal < 500) {
				spin(90);
				enableMotor(BOTH_MOTORS);
				setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
				setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
			}

			if (left2SensorVal > 500 and right2SensorVal > 500) {
				done = true;
				return;
			}
		}

		// calculate delta time for PID
		unsigned long current_time = millis();
		float dt = (current_time - last_time) / 1000.0;
		last_time = current_time;

		// PID
		change = update_pid(dt, linePos);
		// Serial.print("\t");
		// Serial.print(change);

		// calculate wheel speeds from PID values
		leftMotorSpeed = BASE_SPEED - change;
		rightMotorSpeed = BASE_SPEED + change;
	}

	// If PID value makes one of the wheel speed go
	// out of bounds, add it to the opposite wheel
	if (leftMotorSpeed < 0) {
		rightMotorSpeed += -leftMotorSpeed;
		leftMotorSpeed = 0;
	}

	if (rightMotorSpeed < 0) {
		leftMotorSpeed += -rightMotorSpeed;
		rightMotorSpeed = 0;
	}

	// Serial.print("\t");
    // Serial.print(leftMotorSpeed);
	// Serial.print("\t");
    // Serial.print(rightMotorSpeed);

	// update wheel speed
	setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
	setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);

    Serial.println();

	// PWM frequency is 500Hz or 2ms for each pulse
	// wait 3ms for pulse to be sent to motor
	// https://energia.nu/guide/foundations/micro/tutorial_pwm/
	// delay(3);
}