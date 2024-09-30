from Custom_Joystick import JoystickReader
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from communication_interfaces.msg import AutoboxControl

class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(AutoboxControl, 'adre_autoboxControl', 1)

        self.declare_parameter('hz', 60.0)
        hz = self.get_parameter('hz').get_parameter_value().double_value

        timer_period = 1.0 / hz  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.joystick = JoystickReader(js_id=1)
        # JS_ID = 0: ROSMASTER DEFAULT CONTROLLER
        # JS_ID = 1: STADIA CONTROLLER

        self.afk = True

    def timer_callback(self):
        # Actualizar el estado del joystick
        self.joystick.update()

        # Capturar valores del joystick
        acc = self.joystick.get_value(param="RK1_UP_DOWN")
        if acc is None:
            acc = 0.0
        steering = self.joystick.get_value(param="RK2_LEFT_RIGHT")
        if steering is None:
            steering = 0.0
        
        # Normalizar las velocidades en el rango [-1.0, 1.0]
        acc = max(-1.0, min(1.0, acc))
        steering = max(-1.0, min(1.0, steering))

        # Crear el mensaje AckermannControl
        msg = AutoboxControl()
        msg.acceleration = float(acc)
        msg.wheelAngle = float(steering)

        # Publicar el mensaje
        if True: # algún presionado
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing Ackermann: Acceleration={msg.acceleration}, Steering={msg.wheelAngle}')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = JoystickPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
