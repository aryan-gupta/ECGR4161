//*******************************************************************
// ServoExample - Run an inexpensive Servo Motor
// James Conrad, 2020-06-10
// Aryan Gupta, 2020-06-13
//*******************************************************************

#undef min
#undef max
#include <algorithm>
#include <Servo.h>

#include "SimpleRSLK.h"

#define ARRAY_LEN(X) (sizeof(X) / sizeof(X[0]))

// Various constants defining the physical characteristics
// of the vehicle
#define WHEEL_DIAM 0.06959 // in meters
#define WHEEL_BASE 0.143 // in meters
#define CNT_PER_REV 360 // Number of encoder (rising) pulses every time the wheel turns completely

#define STRAIGHT_AHEAD_ANGLE 80
#define LEFT_ANGLE 185
#define RIGHT_ANGLE -10

/// PID Const Variables
#define PID_OUT_MIN -25
#define PID_OUT_MAX 25
#define PID_Kp 1 // 0.022
#define PID_Ki 0.0 // 0.0001
#define PID_Kd 0.0 // 0.003
#define PID_SETPOINT 10.0

// default, max, and min speed of the wheels
// the correction speed is used if the encoder
// values are out of sync and a wheel needs to sped up
#define WHEEL_SPEED 15 // 15%
#define MAX_SPEED 25 // 25%
#define MIN_SPEED 11 // 6%
#define CORRECTION_SPEED_DELTA 5 // 5%

// Percent of journey to stop acceleration
// and start deceleration
#define SPEED_RAMP_ACCEL 0.10 // 0% to 10% accelerate
#define SPEED_RAMP_DECEL 0.40 // 40% to 100% decelerate
#define SPEED_STEP_ACCEL 0.25
#define SPEED_STEP_DECEL 0.75

/// Binds a variable to a lower or upper bind
/// I thought it would be used more often, I guess not
#define LOWER_BIND(X, Y) if ((X) < (Y)) { (X) = (Y); }
#define UPPER_BIND(X, Y) if ((X) > (Y)) { (X) = (Y); }

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created

constexpr int DEFAULT_SERVO_POS = 90;
constexpr size_t NUM_SAMPLES = 5;

const int trigPin = 32; //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad

enum DIRECTIONS {
	NORTH,
	EAST,
	WEST,
	SOUTH,

	DIRECTIONS_MASK
};

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

void setup() {    // put your setup code here, to run once:
	// initialize two digital pins as outputs.
	Serial.begin(115200);
	setupRSLK(); // Set up all of the pins & functions needed to

	setupWaitBtn(LP_LEFT_BTN);   // Left button on Launchpad
	setupLed(RED_LED);           // Red LED of the RGB LED

	pinMode(76, OUTPUT);  //RGB LED - P2.1 GREEN_LED
	pinMode(77, OUTPUT);  //RGB LED - P2.2 BLUE_LED
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

	myservo.attach(38);  // attaches the servo on Port 2.4 to the servo object
	myservo.write(DEFAULT_SERVO_POS);    // Send it to the default position

	String btnMsg = "Push left button on Launchpad to start demo.\n";
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
	delay(2000); // wait 2 seconds for user to run away

}

/// Calculates the speed (in percent) the vehicle
/// should travel at using the ramp function
///
/// @param progress The progress of the vehicle in a percentage
/// @param SPEED_RAMP_ACCEL A constant defined var containing the
///        at what percent to stop acceleration
/// @param SPEED_RAMP_DECEL A constant defined var containing the
///        at what percent to stat deceleration
/// @return The speed of the vehicle
uint8_t calc_speed_ramp(float progress) {
	Serial.println();
	Serial.print("progress: ");
	Serial.print(progress);

	if (progress < SPEED_RAMP_ACCEL) { // acceleration ramp
		float slope = float(MAX_SPEED - MIN_SPEED) / SPEED_RAMP_ACCEL;

		Serial.print("\t slope: ");
		Serial.print(slope);

		return slope * progress + MIN_SPEED;
	} else if (progress > SPEED_RAMP_DECEL) { // deceleration ramp
		progress -= (1 - SPEED_RAMP_DECEL);
		float slope = float(MAX_SPEED - MIN_SPEED) / -(SPEED_RAMP_DECEL);

		Serial.print("\t slope: ");
		Serial.print(slope);

		return slope * progress + MAX_SPEED;
	} else { // plauteu section of ramp
		Serial.print("\t slope: ");
		Serial.print("platue");

		return MAX_SPEED;
	}
}

/// Calculates the speed (in percent) the vehicle
/// should travel at using a simple step function.
/// Will travel vehicle slowly for some time, jump to
/// normal speed, then jump back to slow speed for the
/// last leg of the journey
///
/// @param progress The progress of the vehicle in a percentage
/// @param SPEED_STEP_ACCEL A constant defined var containing the
///        at what percent to resume normal speed
/// @param SPEED_STEP_DECEL A constant defined var containing the
///        at what percent to go back to slow speed
/// @return The speed of the vehicle
uint8_t calc_speed_step(float progress) {
	if (progress < SPEED_STEP_ACCEL) {
		return MIN_SPEED;
	} else if (progress > SPEED_STEP_DECEL) {
		return MIN_SPEED;
	} else {
		return WHEEL_SPEED;
	}
}

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
/// @todo Make speed control function optional
///
/// @param cnt The number of encoder counts to keep sync until
/// @param speed_ctrl_func A functional pointer used to determine the
///        base speed of the motors, used to transform the speed thoughout
///        the journey of the vehicle
/// @param WHEEL_SPEED The default wheel speed when no transform is used
void keep_straight_until_cnt(uint16_t cnt, uint8_t (*speed_ctrl_func)(float)) {
	static unsigned long last_time = 0;
	uint16_t leftCount = getEncoderLeftCnt();
	uint16_t rightCount = getEncoderRightCnt();
	uint8_t baseSpeed = WHEEL_SPEED;
	uint8_t leftMotorSpeed = baseSpeed;
	uint8_t rightMotorSpeed = baseSpeed;
	uint8_t change = 0;

	while (leftCount < cnt and rightCount < cnt) {
		uint16_t avgCount = (leftCount + rightCount) / 2;
		float progress = float(avgCount) / cnt;

		uint16_t leftCount = getEncoderLeftCnt();
		uint16_t rightCount = getEncoderRightCnt();

		// first get base speed using the transform function
		//baseSpeed = speed_ctrl_func(progress);

		Serial.print(baseSpeed);
		Serial.print("\t");

		// then adjust the left and right wheel speeds to sync up
		// the wheel. only gets synced up if the encoder counts there
		// is more than 10 counts of difference otherwise reset the
		// wheel speeds back to default
		pauseMotor(BOTH_MOTORS);
		long pulseLength = get_fast_sonar_val();
		resumeMotor(BOTH_MOTORS);
		float centiSonar = float(pulseLength) / 58;

		Serial.print(pulseLength);
		Serial.print("\t");
		Serial.print(centiSonar);
		Serial.print("\t");

		// calculate delta time for PID
		unsigned long current_time = millis();
		float dt = (current_time - last_time) / 1000.0;
		last_time = current_time;

		change = update_pid(dt, centiSonar);
		// Serial.print("\t");
		// Serial.print(change);

		// calculate wheel speeds from PID values
		leftMotorSpeed = WHEEL_SPEED + change;
		rightMotorSpeed = WHEEL_SPEED - change;

		if (leftMotorSpeed < 0) {
			rightMotorSpeed += -leftMotorSpeed;
			leftMotorSpeed = 0;
		}

		if (rightMotorSpeed < 0) {
			leftMotorSpeed += -rightMotorSpeed;
			rightMotorSpeed = 0;
		}

		setMotorSpeed(LEFT_MOTOR,leftMotorSpeed);
		setMotorSpeed(RIGHT_MOTOR,rightMotorSpeed);

		Serial.println();
	}
}

/// Keeps the motor in sync using the encoders until both
/// encoders have reached \param cnt count. Also will
/// run a speed control function to adjust the speed as it
/// travels
///
/// @todo Make speed control function optional
///
/// @param cnt The number of encoder counts to keep sync until
/// @param speed_ctrl_func A functional pointer used to determine the
///        base speed of the motors, used to transform the speed thoughout
///        the journey of the vehicle
/// @param WHEEL_SPEED The default wheel speed when no transform is used
void sync_motors_until_cnt(uint16_t cnt, uint8_t (*speed_ctrl_func)(float)) {
	uint16_t leftCount = getEncoderLeftCnt();
	uint16_t rightCount = getEncoderRightCnt();
	uint8_t baseSpeed = WHEEL_SPEED;
	uint8_t leftMotorSpeed = baseSpeed;
	uint8_t rightMotorSpeed = baseSpeed;

	while (leftCount < cnt and rightCount < cnt) {
		uint16_t avgCount = (leftCount + rightCount) / 2;
		float progress = float(avgCount) / cnt;

		// first get base speed using the transform function
		baseSpeed = speed_ctrl_func(progress);

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

long get_single_sonar_val() {
	digitalWrite(trigPin, LOW);            // send low to get a clean pulse
    delayMicroseconds(5);                  // let it settle
    digitalWrite(trigPin, HIGH);           // send high to trigger device
    delayMicroseconds(10);                 // let it settle

    return pulseIn(echoPin, HIGH);
}

long get_sonar_val() {
  long samples[NUM_SAMPLES];
  long inches;
  long centimeters;

  for (int i = 0; i < NUM_SAMPLES; ++i) {
    samples[i] = get_single_sonar_val();
    delayMicroseconds(20);
  }

  std::sort(samples, samples + NUM_SAMPLES);

  return samples[(NUM_SAMPLES / 2) + 1];
}

long get_fast_sonar_val() {
	static long lastPulseIn = 0;

	long pulseIn = get_single_sonar_val();

	// if its anything other than 0 or 500 then
	// its a good value
	if (pulseIn == 0) {
		return lastPulseIn;
	}

	if (pulseIn == 500) {
		return lastPulseIn;
	}

	lastPulseIn = pulseIn;
	return pulseIn;
}

float get_sonar_dist() {
	float pulseLength = get_sonar_val();
	// convert pulse length to centimeters
	// then to meters
	return pulseLength / 58 / 100;
}

void servo_goto(int new_pos) {
  static int pos = DEFAULT_SERVO_POS;

  int direction = (new_pos < pos)? -1 : 1;

  for (; pos != new_pos; pos += direction) {
     myservo.write(pos);  // tell servo to go to position in variable 'pos'
     delay(15);           // waits 15ms for the servo to reach the position
  }
}

/// Drives the vehicle straight \param meters number of meters
///
/// @param meters The number of meters to travel straight. Use
///        a negative value to travel backwards
void drive_straight(float meters) {
	servo_goto(LEFT_ANGLE);

	uint16_t encoder_cnt_needed = calc_enoder_count(abs(meters));

	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	if (meters > 0) {
		setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
	} else {
		setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
	}

	enableMotor(BOTH_MOTORS);
	setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);

	keep_straight_until_cnt(encoder_cnt_needed, calc_speed_ramp);

	disableMotor(BOTH_MOTORS);
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
	setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);

	sync_motors_until_cnt(encoder_cnt_needed, calc_speed_step);

	disableMotor(BOTH_MOTORS);
}

void loop() {    // put your main code here, to run repeatedly:
	float dist_to_wall = 0;

	servo_goto(STRAIGHT_AHEAD_ANGLE);
	dist_to_wall = get_sonar_dist();

	float dist_to_travel = dist_to_wall - 0.025;
	drive_straight(dist_to_travel);

	servo_goto(RIGHT_ANGLE);
	dist_to_wall = get_sonar_dist();
	if (dist_to_wall < 0.15) {
		spin(90);
	} else {
		spin(-90);
	}
}
