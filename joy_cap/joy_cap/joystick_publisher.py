from Custom_Joystick import JoystickReader
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom.msg import AckermannControl

class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(AckermannControl, 'control', 1)

        self.declare_parameter('hz', 60.0)
        self.declare_parameter('js_id', 1)  # Declarar el parámetro js_id con valor predeterminado 1

        hz = self.get_parameter('hz').get_parameter_value().double_value
        js_id = self.get_parameter('js_id').get_parameter_value().integer_value  # Obtener el valor del parámetro js_id

        timer_period = 1.0 / hz  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joystick = JoystickReader(js_id=js_id)  # Usar el valor del parámetro js_id para inicializar JoystickReader
        # JS_ID = 0: ROSMASTER DEFAULT CONTROLLER
        # JS_ID = 1: STADIA CONTROLLER

        self.prev_acc = None
        self.prev_steering = None

    def timer_callback(self):
        # Actualizar el estado del joystick
        self.joystick.update()

        # Capturar valores del joystick
        acc = self.joystick.get_value(param="RK1_UP_DOWN")
        if acc is None:
            acc = 0.0
        steering = -self.joystick.get_value(param="RK2_LEFT_RIGHT") # negativo
        if steering is None:
            steering = 0.0
        
        # Normalizar las velocidades en el rango [-1.0, 1.0]
        acc = max(-1.0, min(1.0, acc))
        steering = max(-1.0, min(1.0, steering))

        # Comparar con los valores anteriores y publicar solo si hay cambios
        if acc != self.prev_acc or steering != self.prev_steering:
            # Crear el mensaje AckermannControl
            msg = AckermannControl()
            msg.acc = float(acc)
            msg.steering = float(steering)

            # Publicar el mensaje
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing Ackermann: Acceleration={msg.acc}, Steering={msg.steering}')

            # Actualizar los valores anteriores
            self.prev_acc = acc
            self.prev_steering = steering

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = JoystickPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()