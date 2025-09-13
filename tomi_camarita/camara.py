#!/usr/bin/env python3
'''
Hello to the world from ev3dev.org
'''

import os
import sys
import time

from pybricks.hubs import EV3Brick                 # Control del ladrillo EV3
from pybricks.ev3devices import Motor, GyroSensor  # Motores y giroscopio
from pybricks.parameters import Port, Direction    # Puertos y direcciones de giro
from pybricks.tools import wait, StopWatch         # Pausa y cronómetro
from pybricks.robotics import DriveBase            # Control de movimiento tipo coche
from Felipe_Funciones import Funciones
# Connect the light sensor array to port 'in1'

from pixycamev3.pixy2 import Pixy2

pixy2 = Pixy2(port=4, i2c_address=0x54)

# Initialize the EV3 display
ev3 = EV3Brick()

# Motores de conducción y accesorios
motor_izquierdo = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # Rueda izquierda
motor_derecho = Motor(Port.B)                                # Rueda derecha
pala = Motor(Port.C, Direction.CLOCKWISE)                    # Mecanismo pala
brazo = Motor(Port.D, Direction.CLOCKWISE)                   # Mecanismo brazo

# Base de conducción (DriveBase)
robot = DriveBase(motor_izquierdo, motor_derecho, wheel_diameter=42, axle_track=220)


box_amarillo=0
box_roja=0
box_blanca=0
box_verde=0

def detectar_bloques():
    debug_print("-- Start detection -- ")
    #move_distance_cm(23, 20) # Movimiento hasta el Primer box
    Funciones.mover_con_pid_sin_reiniciar(23, 90)
    time.sleep(1)
    # Repito la accion de detectar y avanzar 6 veces
    for i in range(6):
        pieza = detect_signature()
        debug_print("Lugar: ", {i+1}, "Deteccion: ", pieza)
        if pieza != None:
            if pieza== "white_box":
                box_blanca=i+1
            elif pieza=="green_box":
                box_verde=i+1
            elif pieza=="red_box":
                box_roja=i+1
            elif pieza=="yellow_box":
                box_amarillo=i+1

        #detecciones.append(pieza)
        if i < 5: # Solo avanzo 5 veces, la sexta no
            #move_distance_cm(9.3, 20)
            Funciones.mover_con_pid_sin_reiniciar(9.3, 90)
            time.sleep(1)

    # Finalizo el bucle y limpio la lista de detecciones
    #detecciones = ["Vacio" if x is None else x for x in detecciones]
    debug_print("------------------------------")
    debug_print("amarillo: ",box_amarillo)
    debug_print("rojo: ",box_roja)
    debug_print("verde: ",box_verde)
    debug_print("blanco: ",box_blanca)
    

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

# Estos colores pueden cambiar, debe coincidir con los configurados en PixyMon
colores = {'1': "white_box", '2': "red_box", '3': "yellow_box", '4': "green_box"}

def debug_print(*args, **kwargs):

    '''
    Print debug messages to stderr.
    This shows up in the output panel in VS Code.
    '''
    print(*args, **kwargs, file=sys.stderr)

# Función para detectar la firma y devolver el color machea con la lista de colores, OJO !
# Si no detecta nada, devuelve None
def detect_signature():
    nr_blocks, blocks = pixy2.get_blocks(15, 1) # Con el valor 15 devuelve la deteccion de hasta el 4 signature
    try:
        #debug_print("nro: ", nr_blocks, "blocks: ", blocks[0])
        if nr_blocks >= 1:
            sig = blocks[0].sig
            x = blocks[0].x_center
            y = blocks[0].y_center
            w = blocks[0].width
            h = blocks[0].height
            #debug_print("signature: ", colores.get(str(sig)))
            #debug_print("X center: ", x)
            #debug_print("Y center: ", y)
            #debug_print("width: ", w)
            #debug_print("height: ", h)
            return colores.get(str(sig))
    except:
        pass
    #debug_print("----------------------------------------")

detecciones = []
"""
if __name__ == '__main__':

    # Get version
    version = pixy2.get_version()
    debug_print('Hardware: ', version.hardware)
    debug_print('Firmware: ', version.firmware)
    resolution = pixy2.get_resolution()
    debug_print('Frame width:  ', resolution.width)
    debug_print('Frame height: ', resolution.height)
    pixy2.set_lamp(1, 1) # Turn on the Pixy2 lamp
"""
