import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom.msg import Odometry
from Rosmaster_Lib import Rosmaster
import math


class SpeedPublisher(Node):

    def __init__(self):
        super().__init__('odo_publisher') 
        self.publisher_ = self.create_publisher(Odometry, 'odo', 1)

        self.declare_parameter('hz', 30.0)
        self.declare_parameter('print_info', False)
        hz = self.get_parameter('hz').get_parameter_value().double_value
        self.print_info = self.get_parameter('print_info').get_parameter_value().bool_value

        timer_period = 1.0 / hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.robot = Rosmaster(debug=False)
        self.robot.create_receive_threading()
        self.max_speed = 0
        self.max_yaw = 0

    def convertir_a_radianes(self, valor):
        angulo_grados = (valor + 0.045) * 1000 + 45
        angulo_radianes = (angulo_grados - 90) * (math.pi / 180)
        return float(angulo_radianes)

    def timer_callback(self):
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        motion = self.robot.get_motion_data()
        speed = float(motion[0])
        angle = self.convertir_a_radianes(motion[1])
        
        msg.speed = speed
        msg.steering = angle

        self.max_speed = max(speed, self.max_speed)
        self.max_yaw = max(angle, abs(self.max_yaw))
        self.publisher_.publish(msg)
        
        if self.print_info:
            self.get_logger().info(f'Publishing Ego: speed={msg.speed} m/s, yaw={msg.steering} rad')

def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()
    rclpy.spin(speed_publisher)
    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()