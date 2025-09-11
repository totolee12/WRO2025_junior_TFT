#!/usr/bin/env python3
'''
Hello to the world from ev3dev.org
'''

import os
import sys
import time

from ev3dev2.sensor.lego import Sensor
from ev3dev2.display import Display
from ev3dev2.motor import MediumMotor, OUTPUT_A, OUTPUT_B
from ev3dev2 import fonts  # Import the fonts module

# Connect the light sensor array to port 'in1'
light_sensor = Sensor(address='ev3-ports:in1')

# Initialize the EV3 display
display = Display()

motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)

def line_follower_pid(base_speed=20, kp=10.0, ki=0.0, kd=0.5):
    '''
    PID line follower using weighted average of all sensors.
    The robot tries to keep the line centered between sensors 3 and 4.
    Runs indefinitely (while True).
    '''
    num_sensors = 8
    positions = [i for i in range(num_sensors)]
    integral = 0
    last_error = 0
    last_time = time.time()
    while True:
        try:
            values = [light_sensor.value(i) for i in range(num_sensors)]
        except Exception:
            debug_print("Error reading light sensor values")
            continue
        weights = [100 - v for v in values]
        total_weight = sum(weights)
        if total_weight == 0:
            error = 0
        else:
            line_pos = sum(w * p for w, p in zip(weights, positions)) / total_weight
            error = line_pos - 3.5
        now = time.time()
        dt = now - last_time if last_time else 0.01
        integral += error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0
        correction = kp * error + ki * integral + kd * derivative
        last_error = error
        last_time = now
        left_speed = base_speed + correction
        right_speed = base_speed - correction
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        motor_left.on(-left_speed)
        motor_right.on(right_speed)
        time.sleep(0.01)

def debug_print(*args, **kwargs):
    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

#debug_print(light_sensor.value(7)) # Print the value of the first sensor
def line_follower_pid_time(duration_sec=10, base_speed=20, kp=10.0, ki=0.0, kd=0.5):
    '''
    PID line follower using weighted average of all sensors, runs for a set duration.
    '''
    num_sensors = 8
    positions = [i for i in range(num_sensors)]
    integral = 0
    last_error = 0
    last_time = time.time()
    end_time = time.time() + duration_sec
    while time.time() < end_time:
        try:
            values = [light_sensor.value(i) for i in range(num_sensors)]
        except Exception:
            debug_print("Error reading light sensor values")
            continue
        weights = [100 - v for v in values]
        total_weight = sum(weights)
        if total_weight == 0:
            error = 0
        else:
            line_pos = sum(w * p for w, p in zip(weights, positions)) / total_weight
            error = line_pos - 3.5
        now = time.time()
        dt = now - last_time if last_time else 0.01
        integral += error * dt
        derivative = (error - last_error) / dt if dt > 0 else 0
        correction = kp * error + ki * integral + kd * derivative
        last_error = error
        last_time = now
        left_speed = base_speed + correction
        right_speed = base_speed - correction
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        motor_left.on(-left_speed)
        motor_right.on(right_speed)
        time.sleep(0.01)
    motor_left.off(brake=True)
    motor_right.off(brake=True)


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
        values = [light_sensor.value(i) for i in range(num_sensors)]
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
    # Example: follow a line for 10 seconds at base speed 30 with PID
    line_follower_pid(base_speed=20, kp=30.0, ki=0.0, kd=5)
