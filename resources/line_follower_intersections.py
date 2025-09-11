
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

def debug_print(*args, **kwargs):
    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


def calculate_line_error(values):
    positions = [-400, -300, -200, -100, 100, 200, 300, 400]
    weights = [100 - v for v in values]  # Invert: black = high weight
    total_weight = sum(weights)
    if total_weight == 0:
        return 0  # Avoid division by zero
    error = sum(w * p for w, p in zip(weights, positions)) / total_weight
    return int(error) / 100


def read_line_array():
    num_sensors = 8
    values = [light_sensor.value(i) for i in range(num_sensors)]
    return values


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
        debug_print(values, "-->", calculate_line_error(values))
        # time.sleep(0.5)

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
def line_follower_with_intersection(base_speed=20, kp=1.0, ki=0.0, kd=0.2, threshold=30):
    '''
    PID line follower with intersection detection and counting.
    Stops for 2 seconds at each intersection and counts them.
    threshold: value below which sensor is considered 'black'.
    '''
    integral = 0
    last_error = 0
    last_time = time.time()
    full_intersection_count = 0
    left_intersection = 0
    right_intersection = 0
    while True:
        try:
            values = [light_sensor.value(i) for i in range(8)]
        except Exception:
            debug_print("Error reading light sensor values")
            continue
        # Calculate error using your existing function
        error = calculate_line_error(values)
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
        # Intersection detection
        full_black = all(v < threshold for v in values)
        if full_black:
            motor_left.off(brake=True)
            motor_right.off(brake=True)
            full_intersection_count += 1
            debug_print("Intersection Count:", full_intersection_count)
            # Here you can define movement for eac intersection detected
            if full_intersection_count == 1:
                pass
            if full_intersection_count == 2:
                pass
            #This instrcctions are used ony to test
            time.sleep(2)
            move_distance_cm(3, 20)
        left_black = all(values[i] < threshold for i in range(4))
        if left_black:
            motor_left.off(brake=True)
            motor_right.off(brake=True)
            left_intersection +=1
            debug_print("Intersection LEFT Count:", left_intersection)
            # Here you can define movement for eac intersection detected
            if left_intersection == 1:
                pass
            if left_intersection == 2:
                pass
            #This instrcctions are used ony to test
            time.sleep(2)
            move_distance_cm(3, 20)
        right_black = all(values[i] < threshold for i in range(4,8))
        if right_black:
            motor_left.off(brake=True)
            motor_right.off(brake=True)
            right_intersection +=1
            debug_print("Intersection RIGHT Count:", right_intersection)
            # Here you can define movement for eac intersection detected
            if right_intersection == 1:
                pass
            if right_intersection == 2:
                pass
            #This instrcctions are used ony to test
            time.sleep(2)
            move_distance_cm(3, 20)
        time.sleep(0.01)


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

if __name__ == '__main__':
    line_follower_with_intersection(base_speed=20, kp=30.0, ki=0.0, kd=5, threshold=30)
