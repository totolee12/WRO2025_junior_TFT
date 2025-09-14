#!/usr/bin/env python3
'''
Hello to the world from ev3dev.org   dfgdfhdfg
'''

from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import GyroSensor
from time import sleep

motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)
gyro = GyroSensor()

def turn_to_angle(target_angle, speed=20):
    initial_angle = gyro.angle
    if target_angle > 0:
        while gyro.angle - initial_angle < target_angle:
            motor_left.on(-speed)
            motor_right.on(-speed)
        motor_left.off()
        motor_right.off()
        # Correction step
        while abs(gyro.angle - initial_angle - target_angle) > 0.5:
            error = target_angle - (gyro.angle - initial_angle)
            correction_speed = 5 if error > 0 else -5
            motor_left.on(-correction_speed)
            motor_right.on(-correction_speed)
        motor_left.off()
        motor_right.off()
    else:
        while gyro.angle - initial_angle > target_angle:
            motor_left.on(speed)
            motor_right.on(speed)
        motor_left.off()
        motor_right.off()
        # Correction step
        while abs(gyro.angle - initial_angle - target_angle) > 0.5:
            error = target_angle - (gyro.angle - initial_angle)
            correction_speed = 5 if error < 0 else -5
            motor_left.on(correction_speed)
            motor_right.on(correction_speed)
        motor_left.off()
        motor_right.off()
while True:
    # Turn 90 degrees to the right
    turn_to_angle(90)
    sleep(2)
    # Turn back to original position
    turn_to_angle(-90)
    sleep(2)
