import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.publisher = self.create_publisher(CompressedImage, 'cam', 1)
        
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('ds', 1.0)
        self.declare_parameter('print_info', False)

        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.print_info = self.get_parameter('print_info').get_parameter_value().bool_value

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.cap = cv2.VideoCapture('/dev/video0')
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            downscale = self.get_parameter('ds').get_parameter_value().double_value
            width = 300.0
            height = 300.0

            frame = cv2.resize(frame, (int(width/downscale), int(height/downscale)))

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg").data

            self.publisher.publish(msg)
            
            if self.print_info:
                self.get_logger().info(f'Compressed image sent on 3 topics')
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