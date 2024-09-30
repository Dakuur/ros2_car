import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Crear tres publicadores para tres tópicos distintos
        self.publisher_1 = self.create_publisher(CompressedImage, 'adre_sekonix120degCompressed_resized', 1)
        self.publisher_2 = self.create_publisher(CompressedImage, 'adre_sekonix60degCompressed_resized', 1)
        self.publisher_3 = self.create_publisher(CompressedImage, 'adre_topviewFrontCompressed_resized', 1)
        
        self.declare_parameter('fps', 30.0)  # Declaramos el parámetro 'fps' con un valor por defecto de 30.0
        fps = self.get_parameter('fps').get_parameter_value().double_value
        self.get_logger().info(f'FPS set to {fps}')

        self.declare_parameter('ds', 1.0)
        
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'Publishing at {fps} fps')

        self.cap = cv2.VideoCapture('/dev/video0')
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            downscale = self.get_parameter('ds').get_parameter_value().double_value # 300 x 300 ???
            #width = int(frame.shape[1] / downscale)
            #height = int(frame.shape[0] / downscale)

            width = 300.0
            height = 300.0

            frame = cv2.resize(frame, (int(width/downscale), int(height/downscale)))  # Resizing based on the downscale factor

            # Convert to compressed image
            msg = CompressedImage()
            #compression_params = [cv2.IMWRITE_JPEG_QUALITY, 30]
            #_, buffer = cv2.imencode(".jpg", frame, compression_params)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            #msg.data = buffer.tobytes()
            msg.data = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg").data

            # Publicar en los tres tópicos
            self.publisher_1.publish(msg)
            self.publisher_2.publish(msg)
            self.publisher_3.publish(msg)
            
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
