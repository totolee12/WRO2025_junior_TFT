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

# Connect the light sensor array to port 'in1'
light_sensor = Sensor(address='ev3-ports:in1')

# Initialize the EV3 display
display = Display()

def debug_print(*args, **kwargs):
    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

debug_print(light_sensor.value(7)) # Print the value of the first sensor

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
    main1()
