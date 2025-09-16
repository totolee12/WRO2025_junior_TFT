#!/usr/bin/env pybricks-micropython
from Conexiones.connections import *
from Felipe_Funciones import Funciones
from tomi_camarita.camara import detectar_bloques



if __name__ == "__main__":
    Funciones.mover_con_pid_sin_reiniciar(270,0)    #Avanza al primer punto
    Funciones.giro_izq(-90)                         #gira 90 a la izquierda
    Funciones.mover_con_pid_sin_reiniciar(340,-90)  #avanza hacia el rover
    Funciones.giro_izq(-180)                        #gira mirando a las pelotitas
    Funciones.mover_con_pid_sin_reiniciar(-90,-180) #retrocede llendo hacia el rover
    Funciones.bajar_brazo(95)                       #baja el brazo
    Funciones.mover_con_pid_sin_reiniciar(100,-180) #avanza para bajar el ala del rover
    Funciones.bajar_pala(260)                       #baja la pala
    Funciones.subir_brazo(85)                       #sube el brazo
    Funciones.mover_con_pid_sin_reiniciar(200,-180) #avanza a juntar la primera pelota
    Funciones.mover_con_pid_sin_reiniciar(-100,-180)#retrocede para acomodar
    Funciones.subir_pala(80)                        #sube pala y acomoda la pelota
    Funciones.bajar_pala(80)                        #vuelve a bajar pala
    Funciones.mover_con_pid_sin_reiniciar(100,-180) #va a juntar la otra pelota
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
    #Funciones.mover_con_pid_sin_reiniciar(-50,0)   #por si falta para termina de abrir
    Funciones.subir_pala(80)                        #deposita las pelotitas
    Funciones.mover_con_pid_sin_reiniciar(200,0)    #se aleja 20 cm
    Funciones.giro_izq(-20)                         #gira para acomodarse
    Funciones.mover_con_pid_sin_reiniciar(250,-20)  #abanza para ir a los bloques de colores
    Funciones.giro_der(90)                          #gira para ir a los bloques de colores
    Funciones.mover_con_pid_sin_reiniciar(-100,90)  #retrocede y acomoda con la pared
    detectar_bloques()                              #recoge todos los valores de color
    Funciones.mover_con_pid_sin_reiniciar(50,90)    #avanza hacia dron
    Funciones.giro_der(-180)                        #gira para ir hasta el dron
    Funciones.mover_con_pid_sin_reiniciar(850,-180) #abanza para dejar el dron
    Funciones.mover_con_pid_sin_reiniciar(-850,-180)#retrocede un poco para acomodarse
    Funciones.giro_der(-90)                         #gira para acomodarse contra la pared
    Funciones.mover_con_pid_sin_reiniciar(-100,-90) #retrocede y se acomoda
    Funciones.mover_con_pid_sin_reiniciar(30,-90)   #se mueve adelante para buscar bloques
    Funciones.bajar_pala(75)                        #baja la pala




















