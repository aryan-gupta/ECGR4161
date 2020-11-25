#define WHEEL_DIAM 0.1 // meters
#define WHEEL_BASE 0.2 // meters
#define MY_PI 3.14159 // PI and M_PI could clash with libraries
#define MOTOR_WHEEL_RATIO 48
#define MOTOR_ENCODER_CNT 4
#define WHEELSPEED 15 // 15%

float calc_degrees_to_distance (float degrees) {
    float wheel_circum = WHEEL_DIAM * MY_PI;
    float angle_ratio = degrees / 360.0;
    return angle_ratio * wheel_circum;
}

uint16_t calc_enoder_count (float distance) {
    float wheel_circum = WHEEL_DIAM * MY_PI;
    float wheel_rotations = distance / wheel_circum;
    float motor_rotations = wheel_rotations * MOTOR_WHEEL_RATIO;
    return uint16_t( motor_rotations * MOTOR_ENCODER_CNT );
}

void Pivot120DegreesCCW (void) {
    uint16_t totalCount = 0;

    // transform turn angle into encoder count
    float deg120TurnDistance = calc_degrees_to_distance(120);
    uint16_t deg120TurnEncoderCnt = calc_enoder_count(deg120TurnDistance);

    // Set the encoder pulses count back to zero 
    resetRightEncoderCnt();
    
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    enableMotor(RIGHT_MOTOR);
    setMotorSpeed(RIGHT_MOTOR, WHEELSPEED);

    // Let the motors run until the encoder count is what we need it
    // to be
    while(totalCount < deg120TurnEncoderCnt) {
        totalCount = getEncoderLeftCnt();
        Serial.println(totalCount);
    }

    disableMotor(RIGHT_MOTOR);

    delay(1000);
}
