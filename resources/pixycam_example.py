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

from pixycamev3.pixy2 import Pixy2

pixy2 = Pixy2(port=4, i2c_address=0x54)

# Initialize the EV3 display
display = Display()

motor_left = MediumMotor(OUTPUT_A)
motor_right = MediumMotor(OUTPUT_B)

colores = {'1': "blue_box", '2': "red_box", '3': "yellow_box", '4': "green_box"}

def debug_print(*args, **kwargs):
    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)


if __name__ == '__main__':

    # Get version
    version = pixy2.get_version()
    debug_print('Hardware: ', version.hardware)
    debug_print('Firmware: ', version.firmware)
    resolution = pixy2.get_resolution()
    debug_print('Frame width:  ', resolution.width)
    debug_print('Frame height: ', resolution.height)
    pixy2.set_lamp(1, 0)
    while True:
        # with value 15 get until 4 signature and 1 for only first block of detection
        nr_blocks, blocks = pixy2.get_blocks(15, 1)
        try:
            #debug_print("nro: ", nr_blocks, "blocks: ", blocks[0])
            if nr_blocks >= 1:
                sig = blocks[0].sig
                x = blocks[0].x_center
                y = blocks[0].y_center
                w = blocks[0].width
                h = blocks[0].height
                debug_print("signature: ", colores.get(str(sig)))
                debug_print("X center: ", x)
                debug_print("Y center: ", y)
                debug_print("width: ", w)
                debug_print("height: ", h)
        except:
            pass
        debug_print("----------------------------------------")
        time.sleep(0.5)
