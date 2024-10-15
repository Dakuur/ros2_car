from Rosmaster_Lib import Rosmaster
import rclpy
from rclpy.node import Node
from custom.msg import AckermannControl, Odometry
import math
from time import time, sleep
import signal

class ControlSubscriber(Node):

    def __init__(self):
        super().__init__('control_subscriber')

        # OUTPUT (JOYSTICK OR AI)
        self.subscription = self.create_subscription(
            AckermannControl,
            'control',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        # ODOMETRY
        self.egomaster_subscription = self.create_subscription(
            Odometry,
            'odo',
            self.egomaster_callback,
            1)
        self.egomaster_subscription  # prevent unused variable warning

        # Inicializar el robot
        self.robot = Rosmaster(debug=False)
        #self.robot.create_receive_threading()
        self.robot.set_colorful_effect(0, 5) 

        """self.robot.set_pwm_servo(servo_id=1, angle=45)
        sleep(0.5)
        self.robot.set_pwm_servo(servo_id=1, angle=90)"""

        self.speed = 0  # Velocidad actual real en m/s
        self.yaw = 0 # Yaw actual real en radianes

        self.steering = 0  # Ángulo de dirección
        self.acc = 0  # Última aceleración recibida

        self.previous_time = time()
        self.emergency = False

        # Crear un timer para actualizar y enviar la velocidad constantemente
        hz = 30.0
        self.timer = self.create_timer(1.0 / hz, self.update_speed)

    def listener_callback(self, msg):
        # Actualizar la aceleración y el ángulo de dirección recibidos
        self.acc = msg.acc * 3.6  # Convertir a m/s^2
        self.steering = -msg.steering

        self.get_logger().info(f"Received command: Acceleration={msg.acc:.3f}, Steering={self.steering:.3f}")

    def egomaster_callback(self, msg):
        # Guardar el valor recibido de adre_egomaster en self.speed
        self.speed = msg.speed
        self.yaw = msg.steering
    
    def update_speed(self):
        # Calcular el tiempo transcurrido desde la última actualización
        current_time = time()
        time_diff = current_time - self.previous_time
        self.previous_time = current_time

        # Calcular la nueva velocidad usando la última aceleración recibida
        speed = self.speed + self.acc * time_diff
        self.get_logger().info(f"{speed:.3f}m/s = {self.speed:.3f}m/s + {self.acc:.3f}m/s2 * {time_diff:.3f}s")

        # Limitar la velocidad mínima a 0 si la aceleración es negativa
        if speed < 0:
            speed = 0.0

        limited_speed = min(1.8, speed)  # Limitar a la velocidad máxima de 1.8 m/s

        if float(self.acc) == float(-3.6) and not self.emergency:  # Emergency stop
            self.robot.set_car_motion(0, 0, 0)
            self.robot.set_colorful_effect(effect=3, speed=0, parm=255)  # Activar efecto de color
            self.emergency = True
            print("Emergency stop, stopping for a few seconds")
            sleep(3)
            self.end_emergency()  # Llamar a end_emergency después de la pausa

        # Enviar los comandos de velocidad al robot
        self.robot.set_car_motion(limited_speed, self.steering * 0.045, 0)

        if self.steering == float(0):
            self.robot.set_pwm_servo(servo_id=1, angle=90)

        """self.robot.set_car_run(state=1, speed=limited_speed*(100/1.8), adjust=True)
        angle = self.steering_to_degrees(self.steering)
        self.robot.set_pwm_servo(servo_id=1, angle=angle)"""

        self.get_logger().info(f"Car speed set to {limited_speed:.3f} m/s")

    def end_emergency(self):
        self.robot.set_colorful_effect(0, 5)  # Desactivar efecto de color
        self.emergency = False

    def stop_car(self):
        self.get_logger().info("Stopping car...")
        self.robot.set_car_run(state=1, speed=0, adjust=True)  # Detener el coche
        self.robot.set_pwm_servo(servo_id=1, angle=90)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ControlSubscriber()

    def signal_handler(sig, frame):
        minimal_subscriber.get_logger().info("Ctrl+C pressed. Shutting down...")
        minimal_subscriber.stop_car()
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()