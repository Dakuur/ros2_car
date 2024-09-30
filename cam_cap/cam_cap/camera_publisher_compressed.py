import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera_out', 1)
        self.declare_parameter('ds', 1.0)  # Declaramos el parámetro 'ds' con un valor por defecto de 1.0
        self.get_logger().info(f'Downscale factor set to {self.get_parameter("ds").get_parameter_value().double_value}')

        fps = 60.0
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'Publishing at {fps} fps')

        self.cap = cv2.VideoCapture('/dev/video0')
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            downscale = self.get_parameter('ds').get_parameter_value().double_value
            width = int(frame.shape[1] / downscale)
            height = int(frame.shape[0] / downscale)

            frame = cv2.resize(frame, (width, height))  # Resizing based on the downscale factor

            # Convert to compressed image
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg").data

            self.publisher_.publish(msg)
            self.get_logger().info(f'Compressed image sent with downscale factor: {downscale}')
        else:
            self.get_logger().warning('Failed to capture image')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
