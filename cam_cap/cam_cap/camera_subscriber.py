import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_out',
            self.listener_callback,
            1)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        time_send = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        time_recv = self.get_clock().now().to_msg()
        delay = time_recv.sec + time_recv.nanosec * 1e-9 - time_send
        self.get_logger().info(f'Received an image in {delay} s')

        larger_image = cv2.resize(cv_image, (800, 600))
        cv2.imshow("Camera Subscriber", larger_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
