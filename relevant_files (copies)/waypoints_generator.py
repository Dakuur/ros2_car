import os
import torch
import numpy as np

import rclpy

from sensor_msgs.msg import CompressedImage, PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber

from communication_interfaces.msg import NodeInfo
from tdabus_msgs.msg import Float32

from adre_lib.commons.information.fps_counter import FPSCounter

from adre_ros.commons.transformations.messages_conversion import list_to_statearray, read_img_msg
from adre_lib.perception.interpretation.CILv2_multiview.cilv2_wrapper import CILv2Wrapper
from adre_ros.commons.params.params_reader import ReaderParamsInfo
from adre_ros.commons.node_information.node_info import NodeInfoManager

from cv_bridge import CvBridge

from watchdog_node import WatchdogNode

from custom.msg import AckermannControl, Odometry

class WaypointsGenerator(WatchdogNode):
    """
    Waypoint Generator class for ROS2
    Accepeted arguments:
        topic_in: String value. Message name to subscribe containing the Depth image and Segmented Image.
        topic_out: String value. Message name to publish containing the waypoints coordinate.
        config_file: String value. Configuration file.
        weights_file: String value. Weights file to load the model.
        gpu: Integer value. GPU id to run the model.
    """

    def __init__(self, params):
        node_name = "control"
        super().__init__(node_name, params['logger']['perception'])

        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
            ],
        )
        # update pubisher type
        camera_name1 = params["cilv2_multiview"]['camera_name1']
        camera_name2 = params["cilv2_multiview"]['camera_name2']
        camera_name3 = params["cilv2_multiview"]['camera_name3']
        topic_cam1 = params["cilv2_multiview"]['topic_cam1']
        topic_cam2 = params["cilv2_multiview"]['topic_cam2']
        topic_cam3 = params["cilv2_multiview"]['topic_cam3']
        topic_ego = params["cilv2_multiview"]['topic_ego']
        topic_control = params["cilv2_multiview"]['topic_control']
        topic_action = params["cilv2_multiview"]['topic_action']

        self.image_format1 = params["sensors/cameras"][camera_name1]['format']
        self.camera_version1 = params["sensors/cameras"][camera_name1]['version']
        self.image_custom1 = True if params["sensors/cameras"][camera_name1]['msg'] == 'custom' else False

        self.image_format2 = params["sensors/cameras"][camera_name2]['format']
        self.camera_version2 = params["sensors/cameras"][camera_name2]['version']
        self.image_custom2 = True if params["sensors/cameras"][camera_name2]['msg'] == 'custom' else False

        self.image_format3 = params["sensors/cameras"][camera_name3]['format']
        self.camera_version3 = params["sensors/cameras"][camera_name3]['version']
        self.image_custom3 = True if params["sensors/cameras"][camera_name3]['msg'] == 'custom' else False

        queue_pub = params["cilv2_multiview"]['queue_pub']
        queue_sub = params["cilv2_multiview"]['queue_sub']
        queue_sub_sync = params["cilv2_multiview"]['queue_sub_sync']

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=queue_sub)

        # create publisher
        self.publisher_ = self.create_publisher(AckermannControl, topic_control, queue_pub)
        self.publisher_node_info = self.create_publisher(NodeInfo, "/{}/info".format(node_name), 1)

        # initialize variables
        self.fps_counter = FPSCounter(time_to_display=5, msg="Computing waypoint generation at ")
        self.cv_br = CvBridge()

        # initialize imitation learning parameters
        self.max_speed = params["cilv2_multiview"]['max_speed']
        norm_speed = params["cilv2_multiview"]['norm_speed']
        norm_acc = params["cilv2_multiview"]['norm_acc']
        norm_steer = params["cilv2_multiview"]['norm_steer']
        self.weights_file = params["cilv2_multiview"]['weights_file']
        self.weights_file = params["cilv2_multiview"]['weights_file']
        self.config_file = params["cilv2_multiview"]['config_file']
        self.gpu = params["cilv2_multiview"]['gpu']
        os.environ["CUDA_VISIBLE_DEVICES"] = str(self.gpu)
        self.node_info = NodeInfoManager(self.publisher_node_info)

        # Imitation model initialization
        params = {
                  'norm_acc': norm_acc,
                  'norm_speed': norm_speed,
                  'norm_steer': norm_steer,
                  }
        self._init_model(self.weights_file, self.config_file, params)
        self.get_logger().info("Waypoint generation model initialized")

        # create subscriber
        sub_cam1 = Subscriber(self, CompressedImage, topic_cam1, qos_profile=qos_policy)
        sub_cam2 = Subscriber(self, CompressedImage, topic_cam2, qos_profile=qos_policy)
        sub_cam3 = Subscriber(self, CompressedImage, topic_cam3, qos_profile=qos_policy)
        sub_action = Subscriber(self, Float32, topic_action, qos_profile=qos_policy)
        sub_ego = Subscriber(self, Odometry, topic_ego, qos_profile=qos_policy)

        ats = ApproximateTimeSynchronizer([sub_cam1, sub_cam2, sub_cam3, sub_ego, sub_action], queue_size=queue_sub_sync, slop=1.0)
        ats.registerCallback(self.listener_callback)
        self.get_logger().info("Listening to (%s, %s, %s, %s) topics" % (topic_cam1, topic_cam2, topic_cam3, topic_ego))

    def listener_callback(self, msg_cam1, msg_cam2, msg_cam3, msg_ego, msg_action):
        # self.node_info.start_computation()

        # transform message
        header = msg_cam1.header
        img1 = read_img_msg(msg_cam1, self.image_format1, False, self.cv_br)
        img2 = read_img_msg(msg_cam2, self.image_format2, False, self.cv_br)
        img3 = read_img_msg(msg_cam3, self.image_format3, False, self.cv_br)
        speed = msg_ego.speed # np.sqrt(msg_ego.velX.measurement ** 2 + msg_ego.velY.measurement ** 2 + msg_ego.velZ.measurement ** 2)

        # infer waypoints
        controls, preprocessed_data = self._run_model(img1, img2, img3, speed, msg_action.data)
        # controls['acceleration'] = 0.2

        # control maximum speed
        if self.max_speed > 0. and speed * 3.6 > self.max_speed and controls['acceleration'] > 0:
            controls['acceleration'] = -0.1

        # publish results
        self._publish_control(controls['acceleration'], controls['steer'], header)
        # self.heartbeat()
        # self.node_info.publish_node_info(header.stamp.sec + header.stamp.nanosec / 1e9)

        # compute true fps
        self.fps_counter.update(display=True, ros_node=self)

    def _publish_control(self, acceleration, steer, header):
        control_msg = AckermannControl()
        control_msg.header = header
        control_msg.acc = acceleration
        control_msg.steering = steer

        self.publisher_.publish(control_msg)

    def _init_model(self, weights_file, config_file, params=None):
        # Load model
        self.model = CILv2Wrapper(params=params, model_config=config_file, model_weights=weights_file)

    @torch.no_grad()
    def _run_model(self, img1, img2, img3, speed, direction):
        data = {'img1': img1, 'img2': img2, 'img3': img3, 'speed': speed, 'direction': direction}

        # Prepare input data
        preprocessed_data = self.model.preprocess_data(data)

        # Get predictions
        outputs = self.model.forward(preprocessed_data)

        return outputs, preprocessed_data


def main(args=None):
    rclpy.init(args=args)

    params_reader = ReaderParamsInfo()
    params = params_reader.read_params(["logger", "sensors/cameras", "cilv2_multiview"])

    waypoints_generation_publisher = WaypointsGenerator(params)

    rclpy.spin(waypoints_generation_publisher)

    waypoints_generation_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
