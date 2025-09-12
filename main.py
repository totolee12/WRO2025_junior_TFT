#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick        # Control del ladrillo EV3
from pybricks.ev3devices import Motor, GyroSensor  # Motores y giroscopio
from pybricks.parameters import Port, Direction   # Puertos y direcciones de giro
from pybricks.tools import wait, StopWatch        # Pausa y cronómetro
from pybricks.robotics import DriveBase           # Control de movimiento tipo coche
from Felipe_Funciones import Funciones
from tomi_camarita import camara

# Inicializar ladrillo EV3
ev3 = EV3Brick()

# Motores de conducción y accesorios
motor_izquierdo = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # Rueda izquierda
motor_derecho = Motor(Port.B)                                # Rueda derecha
pala = Motor(Port.C, Direction.CLOCKWISE)                    # Mecanismo pala
brazo = Motor(Port.D, Direction.CLOCKWISE)                   # Mecanismo brazo

# Base de conducción (DriveBase)
robot = DriveBase(motor_izquierdo, motor_derecho, wheel_diameter=42, axle_track=220)

# Control del brazo
def subir_brazo(altura):
    brazo.run_angle(100, -altura)  # Subir brazo
    wait(500)

def bajar_brazo(altura):
    brazo.run_angle(100, altura)   # Bajar brazo
    wait(500)

# Control de la pala
def subir_pala(altura):
    pala.run_angle(150, altura)    # Subir pala
    wait(500)

def bajar_pala(altura):
    pala.run_angle(100, -altura)   # Bajar pala
    wait(500)


# mover_con_pid_sin_reiniciar(distancia_mm, angulo, velocidad=100, kp=1, ki=0.07, kd=0.1):
# giro_der(angulo, velocidad=200):
# giro_izq(angulo, velocidad=200):

Funciones.mover_con_pid_sin_reiniciar(270,0)
Funciones.giro_izq(-90)
Funciones.mover_con_pid_sin_reiniciar(340,-90)
Funciones.giro_izq(-180)
Funciones.mover_con_pid_sin_reiniciar(-90,-180)
Funciones.bajar_brazo(95)
Funciones.mover_con_pid_sin_reiniciar(90,-180)
Funciones.bajar_pala(260)
Funciones.subir_brazo(85)
Funciones.mover_con_pid_sin_reiniciar(150,-180)
Funciones.mover_con_pid_sin_reiniciar(-150,-180)
Funciones.subir_pala(100)
Funciones.bajar_pala(100)
Funciones.mover_con_pid_sin_reiniciar(150,-180)
Funciones.mover_con_pid_sin_reiniciar(-150,-180)
Funciones.giro_der(-90)
Funciones.mover_con_pid_sin_reiniciar(400,-90)
Funciones.giro_izq(0)
Funciones.bajar_brazo(95)
Funciones.mover_con_pid_sin_reiniciar(-150,0)
Funciones.subir_brazo(30)
Funciones.mover_con_pid_sin_reiniciar(-50,0)
Funciones.subir_brazo(30)
Funciones.mover_con_pid_sin_reiniciar(-60,0)
Funciones.subir_brazo(30)
#Funciones.mover_con_pid_sin_reiniciar(-50,0)
Funciones.subir_pala(80)
Funciones.mover_con_pid_sin_reiniciar(200,0)
Funciones.giro_izq(-20)
Funciones.mover_con_pid_sin_reiniciar(250,0)
Funciones.giro_der(90)
Funciones.mover_con_pid_sin_reiniciar(1130,90) #Aca va la camarita tomando valores
Funciones.giro_der(180)
Funciones.mover_con_pid_sin_reiniciar(850,-180)
Funciones.mover_con_pid_sin_reiniciar(-150,-180)
Funciones.giro_der(0)
Funciones.mover_con_pid_sin_reiniciar(700,0)