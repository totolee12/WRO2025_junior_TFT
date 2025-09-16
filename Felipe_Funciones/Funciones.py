#!/usr/bin/env pybricks-micropython
from Conexiones.connections import *

def mover_con_pid_sin_reiniciar(distancia_mm, angulo, velocidad=100, kp=1, ki=0.07, kd=0.1):
    robot.reset()             # Reinicia medición de distancia
    target_angle = angulo     # Ángulo objetivo recto
    error_acum = 0            # Integral PID
    error_prev = 0            # Derivativo PID
    while abs(robot.distance()) < abs(distancia_mm):  # Hasta recorrer la distancia
        error = target_angle - gyro.angle()           # Diferencia de ángulo
        error_acum += error                           # Suma de errores (integral)
        derivada = error - error_prev                 # Cambio de error (derivativo)
        error_prev = error
        correction = (kp * error) + (ki * error_acum) + (kd * derivada)  # PID
        # Avanza o retrocede aplicando la corrección
        robot.drive(velocidad if distancia_mm > 0 else -velocidad, correction)
        wait(5)   # Pausa pequeña para estabilidad
    robot.stop()  # Detiene motores al finalizar


# Funcion para leer el array_sensor de luz en el piso del frente
def leer_array_sensor():
    lecturas = []
    for reg in range(0x42, 0x4A):  # 0x42 to 0x49 inclusive
        value = light_sensor_frontal.read(reg, 1)[0]
        lecturas.append(value)
    return lecturas

def giro_izq_progresivo(angulo, velocidad=200):
    grados = angulo - gyro.angle()
    porcentaje = (10 * grados) / 100
    aumentoV = velocidad / porcentaje
    velocidadRobot = 0
    reloj = StopWatch()
    while gyro.angle() >= angulo and reloj.time() < 6000:
        while gyro.angle() < porcentaje:
            velocidadRobot += aumentoV
            motor_izquierdo.run(-velocidadRobot)  # Izquierda hacia atrás
            motor_derecho.run(velocidadRobot)     # Derecha hacia adelante
        motor_izquierdo.run(-velocidad)
        motor_derecho.run(velocidad)
    motor_izquierdo.stop()
    motor_derecho.stop()
    wait(300)

def giro_izq(angulo, velocidad=200):
    reloj = StopWatch()
    while gyro.angle() >= angulo and reloj.time() < 6000:
        motor_izquierdo.run(-velocidad)
        motor_derecho.run(velocidad)
    motor_izquierdo.stop()
    motor_derecho.stop()
    wait(300)

def giro_der(angulo, velocidad=200):
    reloj = StopWatch()
    while gyro.angle() <= angulo and reloj.time() < 6000:
        motor_izquierdo.run(velocidad)   # Izquierda hacia adelante
        motor_derecho.run(-velocidad)    # Derecha hacia atrás
        wait(1)
    motor_izquierdo.stop()
    motor_derecho.stop()
    wait(300)

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
