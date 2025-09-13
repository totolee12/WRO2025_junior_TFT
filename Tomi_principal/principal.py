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

Funciones.mover_con_pid_sin_reiniciar(270,0)    #Avanza al primer punto
Funciones.giro_izq(-90)                         #gira 90 a la izquierda
Funciones.mover_con_pid_sin_reiniciar(340,-90)  #avanza hacia el rover
Funciones.giro_izq(-180)                        #gira mirando a las pelotitas
Funciones.mover_con_pid_sin_reiniciar(-90,-180) #retrocede llendo hacia el rover
Funciones.bajar_brazo(95)                       #baja el brazo
Funciones.mover_con_pid_sin_reiniciar(100,-180) #avanza para bajar el ala del rover
Funciones.bajar_pala(260)                       #baja la pala
Funciones.subir_brazo(85)                       #sube el brazo
Funciones.mover_con_pid_sin_reiniciar(150,-180) #avanza a juntar la primera pelota
Funciones.mover_con_pid_sin_reiniciar(-150,-180)#retrocede para acomodar
Funciones.subir_pala(100)                       #sube pala y acomoda la pelota
Funciones.bajar_pala(100)                       #vuelve a bajar pala
Funciones.mover_con_pid_sin_reiniciar(150,-180) #va a juntar la otra pelota
Funciones.mover_con_pid_sin_reiniciar(-150,-180)#retrocede para irse a dejarlas
Funciones.giro_der(-90)                         #gira 90 a la derecha
Funciones.mover_con_pid_sin_reiniciar(400,-90)  #avanza a dejar las pelotas
Funciones.giro_izq(0)                           #gira 0 a la izquerda
Funciones.bajar_brazo(95)                       #baja el brazo
Funciones.mover_con_pid_sin_reiniciar(-150,0)   #abre la caja
Funciones.subir_brazo(30)                       #abre la caja
Funciones.mover_con_pid_sin_reiniciar(-50,0)    #abre la caja
Funciones.subir_brazo(30)                       #abre la caja
Funciones.mover_con_pid_sin_reiniciar(-60,0)    #abre la caja
Funciones.subir_brazo(30)                       #abre la caja
#Funciones.mover_con_pid_sin_reiniciar(-50,0)
Funciones.subir_pala(80)                        #deposita las pelotitas
Funciones.mover_con_pid_sin_reiniciar(200,0)    #se aleja 20 cm
Funciones.giro_izq(-20)                         #gira para acomodarse
Funciones.mover_con_pid_sin_reiniciar(250,-20)  #abanza para ir a los bloques de colores
Funciones.giro_der(90)                          #gira para ir a los bloques de colores
Funciones.mover_con_pid_sin_reiniciar(-100,90)  #retrocede y acomoda con la pared



Funciones.giro_der(180)                         #gira para ir hasta el dron
Funciones.mover_con_pid_sin_reiniciar(850,-180) #abanza para dejar el dron
Funciones.mover_con_pid_sin_reiniciar(-150,-180)#retrocede un poco para acomodarse
Funciones.giro_der(0)                           #gira para mirar al frente
Funciones.mover_con_pid_sin_reiniciar(700,0)    #abanza hasta donde estaba el dron



Funciones.giro_izq(-90)
Funciones.mover_con_pid_sin_reiniciar(200,-90)
Funciones.giro_izq(-180)
Funciones.mover_con_pid_sin_reiniciar(150,-180)
Funciones.mover_con_pid_sin_reiniciar(-150,-180)
Funciones.giro_der(690,-90)
