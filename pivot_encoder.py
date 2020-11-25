def calc_degrees_to_distance (degrees):
    wheel_circum = WHEEL_DIAM * MY_PI
    angle_ratio = degrees / 360.0
    return angle_ratio * wheel_circum

def calc_enoder_count (distance):
    wheel_circum = WHEEL_DIAM * MY_PI
    wheel_rotations = distance / wheel_circum
    motor_rotations = wheel_rotations * MOTOR_WHEEL_RATIO
    return motor_rotations * MOTOR_ENCODER_CNT

def Pivot120DegreesCCW ():
    turn_distance = call calc_degrees_to_distance with 120 degrees
    encoder_cnt_needed = call calc_enoder_count with turn_distance

    call reset_encoders
    call start_right_motor

    while currnet_encoder_cnt < encoder_cnt_needed
        currnet_encoder_cnt = call get_current_right_encoder_cnt

    call stop_right_motor

    return