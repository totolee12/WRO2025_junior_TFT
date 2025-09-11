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


# Connect the light sensor array to port 'in1'
light_sensor = Sensor(address='ev3-ports:in1')

# Initialize the EV3 display
display = Display()

# Initialize medium motors on ports A and B
motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)

def move_distance_cm(distance_cm, max_speed):
    '''
    Move both motors the given distance (in centimeters) with acceleration and deceleration.
    max_speed: degrees per second (positive number)
    '''
    import math
    # Wheel and conversion
    wheel_diameter_cm = 3.0
    wheel_circumference_cm = math.pi * wheel_diameter_cm
    degrees_per_cm = 360 / wheel_circumference_cm
    distance_degrees = distance_cm * degrees_per_cm

    # Progressive acceleration/deceleration profile (symmetric, max at midpoint)
    min_speed = 10  # Minimum speed (deg/sec) to start/stop
    if max_speed < min_speed:
        max_speed = min_speed

    # Reset motor positions
    motor_left.position = 0
    motor_right.position = 0

    total = abs(distance_degrees)
    direction = 1 if distance_degrees >= 0 else -1

    def get_speed(pos):
        # Triangular profile: speed rises to max at midpoint, then falls
        if total == 0:
            return min_speed
        midpoint = total / 2
        if pos <= midpoint:
            # Accelerate
            return min_speed + (max_speed - min_speed) * (pos / midpoint)
        else:
            # Decelerate
            return min_speed + (max_speed - min_speed) * ((total - pos) / midpoint)

    while True:
        # Get current position (average of both motors)
        pos = (abs(motor_left.position) + abs(motor_right.position)) // 2
        if pos >= total:
            break
        speed = get_speed(pos)
        # Invert left motor direction for mirrored mounting
        motor_left.on(-direction * speed)
        motor_right.on(direction * speed)
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

#debug_print(light_sensor.value(7)) # Print the value of the first sensor

def main1():
    '''
    The main function of our program
    '''
    # Clear the screen at the start
    display.clear()

    # Get the number of sensors in the array
    num_sensors = 8

    # Main loop to continuously display sensor values
    while True:
        # Clear the display before drawing new bars
        display.clear()

        # Get all sensor values in one go
        values = [light_sensor.value(0), light_sensor.value(1), light_sensor.value(2),
                  light_sensor.value(3), light_sensor.value(4), light_sensor.value(5),
                  light_sensor.value(6), light_sensor.value(7)]
        debug_print(values)

        # Draw the bars and numbers for each sensor using a loop
        for i in range(num_sensors):
            # Get the current sensor value from the list
            value = values[i]

            # Map the sensor value (0-100) to a bar height (0-60 pixels)
            bar_height = int((value / 100) * 60)

            # Define bar width and position
            bar_width = 12
            x_pos = i * 20 + 5
            y_pos = 70 - bar_height

            # Draw the filled bar using the correct parameters
            display.draw.rectangle((x_pos, y_pos, x_pos + bar_width, 64), fill='black')

            # Display the sensor number above the bar
            display.draw.text((x_pos, 0), str(i))

            # Display the numerical value below the bar
            display.draw.text((x_pos,70),str(value))

        # Update the display with the new drawings
        display.update()

        # Add a small delay to control the refresh rate
        time.sleep(0.1)


if __name__ == '__main__':
    # Example usage: move 20 cm at max 600 deg/sec
    move_distance_cm(-100, 50)
    # main1()
