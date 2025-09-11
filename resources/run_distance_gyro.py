#!/usr/bin/env python3
'''
Hello to the world from ev3dev.org
'''

import os
import sys
import time

from ev3dev2.sensor.lego import Sensor
from ev3dev2.display import Display
from ev3dev2 import fonts  # Import the fonts module
from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor.lego import GyroSensor


# Connect the light sensor array to port 'in1'
light_sensor = Sensor(address='ev3-ports:in1')

# Initialize the EV3 display
display = Display()


# Initialize medium motors on ports A and B
motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)

# Initialize gyro sensor (assume port 'in2')
gyro = GyroSensor('ev3-ports:in2')

def move_distance_cm(distance_cm, max_speed):
    '''
    Move both motors the given distance (in centimeters) with acceleration and deceleration.
    max_speed: degrees per second (positive number)
    '''
    import math
    # Wheel and conversion
    wheel_diameter_cm = 4.19
    wheel_circumference_cm = math.pi * wheel_diameter_cm
    degrees_per_cm = 360 / wheel_circumference_cm
    distance_degrees = distance_cm * degrees_per_cm

    # Progressive acceleration/deceleration profile (symmetric, max at midpoint)
    min_speed = 10  # Minimum speed (deg/sec) to start/stop
    # Reserve headroom for correction
    max_motor_speed = 100
    correction_headroom = 10
    if max_speed > max_motor_speed - correction_headroom:
        base_speed = max_motor_speed - correction_headroom
    else:
        base_speed = max_speed
    if base_speed < min_speed:
        base_speed = min_speed

    # Reset motor positions
    motor_left.position = 0
    motor_right.position = 0

    total = abs(distance_degrees)
    direction = 1 if distance_degrees >= 0 else -1

    def get_speed(pos):
        # Trapezoidal profile: 10% accelerate, 80% cruise, 10% decelerate
        if total == 0:
            return min_speed
        accel_dist = total * 0.1
        decel_dist = total * 0.1
        cruise_start = accel_dist
        cruise_end = total - decel_dist
        if pos < accel_dist:
            # Accelerate
            return min_speed + (max_speed - min_speed) * (pos / accel_dist)
        elif pos < cruise_end:
            # Cruise
            return max_speed
        else:
            # Decelerate
            remain = total - pos
            return min_speed + (max_speed - min_speed) * (remain / decel_dist)


    # Use gyro to maintain heading
    target_heading = 30
    """
    try:
        target_heading = gyro.angle
    except Exception:
        pass
    """

    kp_forward = 10.0   # Proportional gain for forward
    kp_backward = 5.0  # Proportional gain for backward (tune as needed)

    while True:
        # Get current position (average of both motors)
        pos = (abs(motor_left.position) + abs(motor_right.position)) // 2
        if pos >= total:
            break
        speed = get_speed(pos)
        # Use the lower of base_speed and current profile speed
        base = min(base_speed, speed)
        # Gyro correction
        error = 0
        try:
            error = gyro.angle - target_heading
        except Exception:
            pass
        if direction > 0:
            correction = kp_forward * error
            left_speed = direction * (base - correction)
            right_speed = direction * (base + correction)
        else:
            correction = kp_backward * error
            # When moving backward, swap correction direction
            left_speed = direction * (base + correction)
            right_speed = direction * (base - correction)
        # Clamp speeds to valid range for ev3dev2 motor (percentage: -100 to 100)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        # Invert left motor direction for mirrored mounting
        motor_left.on(-left_speed)
        motor_right.on(right_speed)
        time.sleep(0.01)

    # Stop motors at the end
    motor_left.off(brake=True)
    motor_right.off(brake=True)

def debug_print(*args, **kwargs):
    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

if __name__ == '__main__':
    # Example usage: move 20 cm at max 600 deg/sec
    move_distance_cm(-150, 80)
    # main1()
