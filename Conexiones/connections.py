
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick        # Control del ladrillo EV3
from pybricks.ev3devices import Motor, GyroSensor  # Motores y giroscopio
from pybricks.parameters import Port, Direction   # Puertos y direcciones de giro
from pybricks.tools import wait, StopWatch        # Pausa y cronómetro
from pybricks.robotics import DriveBase           # Control de movimiento tipo coche
from pybricks.iodevices import I2CDevice
from pixycamev3.pixy2 import Pixy2

# Camarita Pixycam
pixy2 = Pixy2(port=4, i2c_address=0x54)

# Inicializar ladrillo EV3
ev3 = EV3Brick()

# Motores de conducción y accesorios
motor_izquierdo = Motor(Port.A, Direction.COUNTERCLOCKWISE)  # Rueda izquierda
motor_derecho = Motor(Port.B)                                # Rueda derecha
pala = Motor(Port.C, Direction.CLOCKWISE)                    # Mecanismo pala
brazo = Motor(Port.D, Direction.CLOCKWISE)                   # Mecanismo brazo

# Base de conducción (DriveBase)
robot = DriveBase(motor_izquierdo, motor_derecho, wheel_diameter=42, axle_track=220)


# Sensor de Luz Array delantero
light_sensor_frontal = I2CDevice(Port.S1, address = 0x14 >> 1)

# ----------------- Inicializacion de Giroscopo -----------------
# ----------------- Parámetros de calidad -----------------
MAX_ATTEMPTS = 6           # Reintentos de inicialización
SETTLE_MS = 700            # Tiempo para que el sensor "se asiente" tras crear el objeto
SAMPLE_MS = 800            # Ventana para muestrear velocidad (dps) y ver estabilidad
VERIFY_MS = 2000           # Verificación final de deriva de ángulo (~2 s)
RATE_ABS_MAX = 1.5         # |speed()| promedio aceptable en reposo [°/s]
ANGLE_DRIFT_MAX = 2.0      # Deriva total aceptable en la verificación [°]

def _avg_abs(values):
    if not values:
        return 9999.0
    return sum(abs(v) for v in values) / len(values)

def _collect_speeds(gs, duration_ms):
    """Muestrea speed() durante 'duration_ms' y devuelve lista de dps."""
    data = []
    elapsed = 0
    step = 10  # ms
    while elapsed < duration_ms:
        data.append(gs.speed())
        wait(step)
        elapsed += step
    return data

def _measure_drift(gs, duration_ms):
    """Mide deriva total de ángulo en 'duration_ms'."""
    start = gs.angle()
    elapsed = 0
    step = 20  # ms
    while elapsed < duration_ms:
        wait(step)
        elapsed += step
    return gs.angle() - start

def _warm_reset_sequence(gs):
    """
    Secuencia de 'despertar' recomendada:
    - Leer ambos modos (angle y speed) para estabilizar
    - Esperar asentamiento
    - reset_angle(0)
    - Esperar un poco más
    """
    # Lecturas cruzadas para forzar cambio de modo interno
    _ = gs.angle()
    _ = gs.speed()
    wait(100)
    gs.reset_angle(0)
    wait(150)

def init_gyro(port=Port.S1, attempts=MAX_ATTEMPTS):
    """
    Inicializa y valida el giroscopio en 'port'.
    Si algo no cuadra, reintenta creando de nuevo el objeto.
    Devuelve (gyro, ok: bool, msg: str)
    """

    for attempt in range(1, attempts + 1):
        try:
            # Crear objeto nuevo en cada intento
            gs = GyroSensor(port)

            # Dejar que se asiente la electrónica
            wait(SETTLE_MS)

            # Secuencia de “warm reset”
            _warm_reset_sequence(gs)

            # 1) Chequeo de velocidad (sensor quieto)
            speeds = _collect_speeds(gs, SAMPLE_MS)
            avg_abs_rate = _avg_abs(speeds)

            # 2) Chequeo de deriva de ángulo
            gs.reset_angle(0)
            wait(50)
            drift = _measure_drift(gs, VERIFY_MS)
            drift_abs = abs(drift)

            ok = (avg_abs_rate <= RATE_ABS_MAX) and (drift_abs <= ANGLE_DRIFT_MAX)
            msg = "borre mensaje porque daba error"

            # Feedback visual/sonoro útil en banco de pruebas
            print(msg)
            if ok:
                ev3.speaker.beep(1000, 100)
                return gs, True, msg
            else:
                ev3.speaker.beep(300, 80)
                # Pequeña pausa antes del próximo intento
                wait(250)

        except Exception as e:
            print(" borre otro mensaje que daba error")
            # Pausa y reintentar
            wait(300)

    # Si llegamos acá, no se pudo estabilizar.
    return None, False, "No se pudo inicializar el giroscopio tras {attempts} intentos."

# ----------------- Ejemplo de uso -----------------
gyro, ok, info = init_gyro(Port.S2)
