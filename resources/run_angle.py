#!/usr/bin/env python3
import time
from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import GyroSensor

# Initialize motors
motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)

# Initialize gyro sensor on port B
gyro = GyroSensor('ev3-ports:in2')

def turn_to_angle_absolute(target_angle, speed=30):
    '''
    Reset gyro and turn to the specified angle (absolute, from 0)
    '''
    gyro.reset()
    time.sleep(0.1)  # Let gyro settle
    while True:
        current = gyro.angle
        error = target_angle - current
        if abs(error) < 2:
            break
        direction = 1 if error > 0 else -1
        # For in-place turn: left and right motors in opposite directions
        motor_left.on(-direction * speed)
        motor_right.on(-direction * speed)
        time.sleep(0.01)
    motor_left.off(brake=True)
    motor_right.off(brake=True)

def turn_to_angle_relative(delta_angle, speed=30):
    '''
    Turn by a relative angle (does not reset gyro)
    '''
    start_angle = gyro.angle
    target_angle = start_angle + delta_angle
    while True:
        current = gyro.angle
        error = target_angle - current
        if abs(error) < 2:
            break
        direction = 1 if error > 0 else -1
        # For in-place turn: left and right motors in opposite directions
        motor_left.on(-direction * speed)
        motor_right.on(-direction * speed)
        time.sleep(0.01)
    motor_left.off(brake=True)
    motor_right.off(brake=True)

if __name__ == '__main__':
    # Test absolute turn: resets gyro and turns to +90 degrees
    turn_to_angle_absolute(-45, speed=30)
    time.sleep(1)
    # Test relative turn: turns +90 degrees from current position
    turn_to_angle_relative(-45, speed=30)
