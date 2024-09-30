from Rosmaster_Lib import Rosmaster
from time import sleep
import math
# import keyboard

g_bot = Rosmaster(debug=True)
g_bot.create_receive_threading()

def curva_normal(x, mean=0, stddev=1):
    # Función que genera una curva normal con media y desviación estándar dadas
    return (1 / (stddev * math.sqrt(2 * math.pi))) * math.exp(-0.5 * ((x - mean) / stddev) ** 2)

def prueba_motion_curva_normal(g_bot):
    mean = 0  # Media de la curva
    stddev = 1  # Desviación estándar
    steps = 20  # Cantidad de pasos para la curva
    max_speed = 1  # Velocidad máxima en v_x
    time_per_step = 0.2  # Tiempo entre cada paso

    for i in range(steps):
        # Generar el valor de la curva normal
        x = (i - steps / 2) / (steps / 2)  # Valores desde -1 a 1
        normal_value = curva_normal(x, mean=0, stddev=0.5)  # Escalar la curva
        v_x = max_speed * normal_value  # Ajustar la velocidad según la curva
        g_bot.set_car_motion(v_x=v_x, v_y=0, v_z=0)
        sleep(time_per_step)

    # Detener el coche al final
    g_bot.set_car_motion(v_x=0, v_y=0, v_z=0)
    g_bot.set_pwm_servo(servo_id=1, angle=90)

def curva_senoidal(x, amplitude=45, offset=90):
    # Función que genera un movimiento senoidal para los ángulos del servo
    return amplitude * math.sin(x) + offset

def mover_servo_curva_senoidal(g_bot):
    steps = 50  # Cantidad de pasos para un ciclo completo de movimiento
    time_per_step = 0.1  # Tiempo entre cada paso
    
    for i in range(steps):
        # Generar un ángulo usando una curva senoidal
        angle = curva_senoidal(2 * math.pi * (i / steps))  # Oscila entre -45 y +45 con un offset de 90
        g_bot.set_pwm_servo(servo_id=1, angle=angle)
        sleep(time_per_step)

    # Devolver el servo a la posición neutral de 90 grados al final
    g_bot.set_pwm_servo(servo_id=1, angle=90)

def realizar_movimientos_simultaneos(g_bot):
    mean = 0  # Media de la curva
    stddev = 1  # Desviación estándar para la velocidad
    steps = 50  # Cantidad de pasos totales para sincronizar ambos movimientos
    max_speed = 1  # Velocidad máxima en v_x
    time_per_step = 0.05  # Tiempo entre cada paso

    for i in range(steps):
        # Generar el valor de la curva normal para la velocidad
        x = (i - steps / 2) / (steps / 2)  # Valores desde -1 a 1
        normal_value = curva_normal(x, mean=0, stddev=0.5)  # Escalar la curva
        v_x = max_speed * normal_value  # Ajustar la velocidad según la curva
        g_bot.set_car_motion(v_x=v_x, v_y=0, v_z=0)
        
        # Generar el valor de la curva senoidal para el ángulo del servo
        angle = curva_senoidal(2 * math.pi * (i / steps))  # Oscila entre -45 y +45 con un offset de 90
        g_bot.set_pwm_servo(servo_id=1, angle=angle)
        
        # Pausar antes del siguiente paso
        sleep(time_per_step)

    # Detener el coche y el servo al final
    g_bot.set_car_motion(v_x=0, v_y=0, v_z=0)
    g_bot.set_pwm_servo(servo_id=1, angle=90)
"""
def controlar_robot_teclado(g_bot):
    try:
        velocidad_lineal = 0.5  # Velocidad de avance/retroceso
        velocidad_lateral = 0.5  # Velocidad de movimiento lateral (izquierda/derecha)
        servo_incremento = 5  # Incremento del ángulo del servo
        angulo_servo = 90  # Posición inicial del servo
        
        print("Usa las teclas de flechas o WASD para controlar el robot. Q/E para mover el servo.")
        while True:
            if keyboard.is_pressed('w') or keyboard.is_pressed('up'):
                # Avanzar
                g_bot.set_car_motion(v_x=velocidad_lineal, v_y=0, v_z=0)
            elif keyboard.is_pressed('s') or keyboard.is_pressed('down'):
                # Retroceder
                g_bot.set_car_motion(v_x=-velocidad_lineal, v_y=0, v_z=0)
            elif keyboard.is_pressed('a') or keyboard.is_pressed('left'):
                # Desplazarse a la izquierda
                g_bot.set_car_motion(v_x=0, v_y=velocidad_lateral, v_z=0)
            elif keyboard.is_pressed('d') or keyboard.is_pressed('right'):
                # Desplazarse a la derecha
                g_bot.set_car_motion(v_x=0, v_y=-velocidad_lateral, v_z=0)
            else:
                # Detener el coche si no se presiona ninguna tecla
                g_bot.set_car_motion(v_x=0, v_y=0, v_z=0)
                g_bot.set_car_run(state=0, speed=0, adjust=True)

            # Controlar el ángulo del servo con Q y E
            if keyboard.is_pressed('q'):
                # Mover el servo hacia la izquierda
                angulo_servo = max(0, angulo_servo - servo_incremento)
                g_bot.set_pwm_servo(servo_id=1, angle=angulo_servo)
            elif keyboard.is_pressed('e'):
                # Mover el servo hacia la derecha
                angulo_servo = min(180, angulo_servo + servo_incremento)
                g_bot.set_pwm_servo(servo_id=1, angle=angulo_servo)
            
            sleep(0.1)  # Evitar que se sobrecargue el procesador con bucles rápidos
    except KeyboardInterrupt:
        # Detener el coche y resetear el servo a 90 grados al salir
        g_bot.set_car_motion(v_x=0, v_y=0, v_z=0)
        g_bot.set_pwm_servo(servo_id=1, angle=90)
        print("Control interrumpido.")
"""
#controlar_robot_teclado(g_bot)

try:
    while True:
        #g_bot.set_car_run(state=5, speed=20, adjust=True)
        #g_bot.set_motor(0, 20, 0, 20)
        #controlar_robot_teclado(g_bot)
        #realizar_movimientos_simultaneos(g_bot)
        angle = 45
        g_bot.set_pwm_servo(servo_id=1, angle=angle)
        print(f"Angle")
        sleep(0.05)
        angle = 90
        g_bot.set_pwm_servo(servo_id=1, angle=angle)
        print(f"Angle")
        sleep(0.05)
        angle = 135
        g_bot.set_pwm_servo(servo_id=1, angle=angle)
        print(f"Angle")
        sleep(0.05)
except KeyboardInterrupt:
    g_bot.set_car_run(state=0, speed=0, adjust=True) # Parar coche i servo 90º
    pass
exit(0)