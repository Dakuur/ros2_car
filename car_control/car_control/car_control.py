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

        self.subscription = self.create_subscription(
            AckermannControl,
            'control',
            self.listener_callback,
            1)
        self.subscription

        self.egomaster_subscription = self.create_subscription(
            Odometry,
            'odo',
            self.egomaster_callback,
            1)
        self.egomaster_subscription

        self.declare_parameter('max_speed', 1.8)
        self.declare_parameter('max_steer', 1.0)
        self.declare_parameter('print_info', False)

        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steer = self.get_parameter('max_steer').get_parameter_value().double_value
        self.print_info = self.get_parameter('print_info').get_parameter_value().bool_value

        self.robot = Rosmaster(debug=True)
        self.robot.set_colorful_effect(0, 5)
        self.robot.set_akm_default_angle(angle=90, forever=True)

        self.speed = 0
        self.yaw = 0
        self.steering = 0
        self.acc = 0
        self.previous_time = time()
        self.emergency = False

        hz = 30.0
        self.timer = self.create_timer(1.0 / hz, self.update_speed)

    def listener_callback(self, msg):
        self.acc = msg.acc * 3.6
        self.steering = -msg.steering

        if self.print_info:
            self.get_logger().info(f"Received command: Acceleration={msg.acc:.3f}, Steering={self.steering:.3f}")

    def egomaster_callback(self, msg):
        self.speed = msg.speed
        self.yaw = msg.steering
    
    def update_speed(self):
        current_time = time()
        time_diff = current_time - self.previous_time
        self.previous_time = current_time

        speed = self.speed + self.acc * time_diff
        if speed < 0:
            speed = 0.0

        limited_speed = min(self.max_speed, speed)
        limited_steer = max(-self.max_steer, min(self.max_steer, self.steering))

        if float(self.acc) == float(-3.6) and not self.emergency:
            self.stop_car()
            self.robot.set_colorful_effect(effect=3, speed=0, parm=255)
            self.emergency = True
            print("Emergency stop, stopping for a few seconds")
            sleep(3)
            self.end_emergency()

        # SET SPEED AND STEERING
        self.robot.set_car_motion(limited_speed, limited_steer * 0.045, 0)
        #self.robot.set_car_run(state=1, speed=limited_speed*100, adjust=True)

        if self.steering == float(0):
            self.robot.set_pwm_servo(servo_id=1, angle=90)

        if self.print_info:
            self.get_logger().info(f"Car speed set to {limited_speed:.3f} m/s.")

    def end_emergency(self):
        self.robot.set_colorful_effect(0, 5)
        self.emergency = False

    def stop_car(self):
        self.get_logger().info("Stopping car...")
        self.robot.set_car_run(state=1, speed=0, adjust=True)
        self.robot.set_pwm_servo(servo_id=1, angle=90)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = ControlSubscriber()

    try:
        minimal_subscriber.get_logger().info("Node spinning, press Ctrl+C to stop")
        while rclpy.ok():
            rclpy.spin_once(minimal_subscriber, timeout_sec=0.1)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info("Ctrl+C pressed. Shutting down...")
    finally:
        minimal_subscriber.stop_car()
        minimal_subscriber.get_logger().info("Car stopped")
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()