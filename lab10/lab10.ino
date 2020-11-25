
#undef min
#undef max
#include <algorithm>
#include <tuple>
#include <Servo.h>

#include "SimpleRSLK.h"
#define ARRAY_LEN(X) (sizeof(X) / sizeof(X[0]))

// Various constants defining the physical characteristics
// of the vehicle
#define WHEEL_DIAM 0.07 // in meters
#define WHEEL_BASE 0.143 // in meters
#define CNT_PER_REV 360 // Number of encoder (rising) pulses every time the wheel turns completely

#define STRAIGHT_AHEAD_ANGLE 80
#define LEFT_ANGLE 185
#define RIGHT_ANGLE -10

// default, max, and min speed of the wheels
// the correction speed is used if the encoder
// values are out of sync and a wheel needs to sped up
#define WHEEL_SPEED 7 // 15%
#define MAX_SPEED 10 // 25%
#define MIN_SPEED 5 // 6%
#define CORRECTION_SPEED_DELTA 5 // 5%

// Percent of journey to stop acceleration
// and start deceleration
#define SPEED_RAMP_ACCEL 0.10 // 0% to 10% accelerate
#define SPEED_RAMP_DECEL 0.40 // 40% to 100% decelerate
#define SPEED_STEP_ACCEL 0.25
#define SPEED_STEP_DECEL 0.75


#define DEFAULT_SERVO_POS 90
#define NUM_SAMPLES_SONAR 7
#define NUM_SAMPLES_BOX 8

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created


const int trigPin = 32; //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33; //This is Port Pin 5.1 on the MSP432 Launchpad

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
		if (speed_ctrl_func == nullptr) {
			baseSpeed = WHEEL_SPEED;
		} else {
			baseSpeed = speed_ctrl_func(progress);
		}

		leftCount = getEncoderLeftCnt();
		rightCount = getEncoderRightCnt();

		// then adjust the left and right wheel speeds to sync up
		// the wheel. only gets synced up if the encoder counts there
		// is more than 10 counts of difference otherwise reset the
		// wheel speeds back to default
		int abs_diff = abs(int(leftCount) - rightCount);
		if (abs_diff > 5) {
			if (leftCount < rightCount) { // left wheel lagging
				leftMotorSpeed = baseSpeed + CORRECTION_SPEED_DELTA;
				rightMotorSpeed = baseSpeed - CORRECTION_SPEED_DELTA;
			}

			if (rightCount < leftCount) { // right wheel lagging
				leftMotorSpeed = baseSpeed - CORRECTION_SPEED_DELTA;
				rightMotorSpeed = baseSpeed + CORRECTION_SPEED_DELTA;
			}
		} else {
			leftMotorSpeed = baseSpeed;
			rightMotorSpeed = baseSpeed;
		}

		setMotorSpeed(LEFT_MOTOR,leftMotorSpeed);
		setMotorSpeed(RIGHT_MOTOR,rightMotorSpeed);
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
	setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);

	sync_motors_until_cnt(encoder_cnt_needed, nullptr /* calc_speed_ramp */);

	disableMotor(BOTH_MOTORS);
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

	sync_motors_until_cnt(encoder_cnt_needed, calc_speed_ramp);

	disableMotor(BOTH_MOTORS);
}

void servo_goto(int new_pos) {
  static int pos = DEFAULT_SERVO_POS;

  int direction = (new_pos < pos)? -1 : 1;

  for (; pos != new_pos; pos += direction) {
     myservo.write(pos);  // tell servo to go to position in variable 'pos'
     delay(15);           // waits 15ms for the servo to reach the position
  }
}

long get_single_sonar_val() {
	digitalWrite(trigPin, LOW);            // send low to get a clean pulse
    delayMicroseconds(5);                  // let it settle
    digitalWrite(trigPin, HIGH);           // send high to trigger device
    delayMicroseconds(10);                 // let it settle

    long pulseLenth = pulseIn(echoPin, HIGH);

	if (pulseLenth == 0) {
		return get_single_sonar_val();
	} else if (pulseLenth == 500) {
		return get_single_sonar_val();
	} else {
		return pulseLenth;
	}
}

long get_sonar_val() {
  long samples[NUM_SAMPLES_SONAR];
  long inches;
  long centimeters;

  for (int i = 0; i < NUM_SAMPLES_SONAR; ++i) {
    samples[i] = get_single_sonar_val();
    delayMicroseconds(50);
  }

  std::sort(samples, samples + NUM_SAMPLES_SONAR);

  return samples[(NUM_SAMPLES_SONAR / 2) + 1];
}

float get_sonar_dist() {
	float pulseLength = get_sonar_val();
	// convert pulse length to centimeters
	// then to meters
	return pulseLength / 58 / 100;
}

float normalize_angle(float angle) {
	if (angle > 360.0) {
		return normalize_angle(angle - 360.0);
	}

	if (angle < 0.0) {
		return normalize_angle(angle + 360.0);
	}

	return angle;
}

float calc_difference(float a, float b) {
	// https://www.calculatorsoup.com/calculators/algebra/percent-difference-calculator.php
	float num = std::abs(a - b);
	float den = (a + b) / 2;

	return num / den;
}

std::pair<int, int> find_keypoints(float* values) {
	// average even and odd indexed values seperately
	float evens = 0.0, odds = 0.0;
	for (int i = 0; i < NUM_SAMPLES_BOX; ++i) {
		if (i % 2 == 0) {
			evens += values[i];
		} else {
			odds += values[i];
		}
	}

	evens /= NUM_SAMPLES_BOX / 2.0;
	odds  /= NUM_SAMPLES_BOX / 2.0;

	// now that we know the average of the odds and
	// evens, go back though and find the percent
	// difference of each sample. The largest percent
	// difference in the negative (value is smaller
	// than it should) is the direction the box is in
	// any sampled values that are larget than the average
	// set it as -1 so we can ignore it in the next step
	float percentDifference[NUM_SAMPLES_BOX];
	for (int i = 0; i < NUM_SAMPLES_BOX; ++i) {
		if (i % 2 == 0) {
			percentDifference[i] = calc_difference(evens, values[i]);
			percentDifference[i] *= (evens > values[i])? -1 : 1;
		} else {
			percentDifference[i] = calc_difference(odds, values[i]);
			percentDifference[i] *= (odds > values[i])? -1 : 1;
		}
	}

	// now that we know the differences, find the largest
	// in the negative direction and the largest in the
	// positive direction
	int maxIdx = 0, minIdx = 0;
	for (int i = 1; i < NUM_SAMPLES_BOX; ++i) {
		if (percentDifference[i] > percentDifference[maxIdx]) {
			maxIdx = i;
		}

		if (percentDifference[i] < percentDifference[minIdx]) {
			minIdx = i;
		}
	}

	// return the index that had the largest negative percent difference
	return { minIdx, maxIdx };
}

bool calc_if_facing_side(float* values) {
	// average even and odd indexed values seperately
	float evens = 0.0, odds = 0.0;
	for (int i = 0; i < NUM_SAMPLES_BOX; ++i) {
		if (i % 2 == 0) {
			evens += values[i];
		} else {
			odds += values[i];
		}
	}

	evens /= NUM_SAMPLES_BOX / 2.0;
	odds  /= NUM_SAMPLES_BOX / 2.0;

	// the first value (evens) is the direction we are facing
	// if the evens average is less than the odds average then
	// the evens was the sides and odd samples were the corners
	return evens < odds;
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
	servo_goto(82);

	String btnMsg = "Push left button on Launchpad to start demo.\n";
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
	delay(2000); // wait 2 seconds for user to run away
}

void loop() {
	servo_goto(82);

	// Did the vehicle start facing a side

	float sonarValues[NUM_SAMPLES_BOX];
	for (int i = 0; i < NUM_SAMPLES_BOX; ++i) {
		float sonarValue = get_sonar_dist();
		sonarValues[i] = sonarValue;
		spin(360.0 / NUM_SAMPLES_BOX);
		delay(100);
	}

	int boxIndex, exitIndex;
	std::tie(boxIndex, exitIndex) = find_keypoints(sonarValues);
	float angleToBox = (360.0 / NUM_SAMPLES_BOX) * boxIndex;
	float angleToExit = (360.0 / NUM_SAMPLES_BOX) * exitIndex;

	if (boxIndex > (NUM_SAMPLES_BOX / 2)) {
		angleToBox -= 360.0;
	}

	delay(500);
	spin(angleToBox);
	for (int i = 0; i < 3; ++i) {
		digitalWrite(BLUE_LED, HIGH);   // turn the RED LED on (HIGH is the voltage level)
		delay(500);
		digitalWrite(BLUE_LED, LOW);
		delay(500);
	}
	// spin(-angleToBox);

	angleToBox += 360;
	angleToExit = angleToExit - angleToBox;
	delay(500);
	spin(angleToExit);

	delay(500);
	drive_straight(sonarValues[exitIndex]);

	// Serial.println();
	// while (true)
	// {
	// 	for (int i = 0; i < NUM_SAMPLES_BOX; ++i) {
	// 		Serial.print(sonarValues[i]);
	// 		Serial.print(" ");
	// 	}
	// 	Serial.println();
	// 	delay(1000);
	// }

	String btnMsg = "Push left button on Launchpad to start demo.\n";
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
}