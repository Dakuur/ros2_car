import math
import sys
from asyncio import Future

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
from time import time, sleep
import logger as log

from adre_lib.ego.vehicle import Vehicle
from adre_lib.motion_control.path_planning.obstacle_avoidance.obstacle import Obstacle
from adre_lib.commons.coord_sys.switch_reference import global_to_local
from adre_lib.parameters.parameters_parser import Parameters
from adre_ros.commons.params.params_reader import ReaderParamsInfo
from adre_ros.commons.transformations.messages_conversion import autoboxcontrol_to_list, detection2d_to_array, detection3d_to_array, statearray_to_list, msg2pc, state_to_list

from std_msgs.msg import Bool, UInt32
from sensor_msgs.msg import PointCloud2
from communication_interfaces.msg import State, StateArray, Detections2DArray, Detections3DArray, AutoboxControl, AutoboxInfo, Multi2DArray
from gpsrtk_msgs.msg import Position


class ROSManagerVisualization(Node):
    """
    ROS class, which enables to generate a ROS2 Node with a publisher and subscribers.
    """

    def __init__(self, node_name, params):

        super().__init__(node_name)

        self.compressed_images = True
        self.plot_legend = True

        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ('topic_road_left', "/perception/road_borders_left"),
                ('topic_road_right', "/perception/road_borders_right"),
                ('topic_fit_right', "/perception/fit/right_est"),
                ('topic_fit_left', "/perception/fit/left_est"),
                ('topic_left_cropped', "/perception/fit/left_cropped"),
                ('topic_right_cropped', "/perception/fit/right_cropped"),
                ('topic_rtk', "/gpsrtk/position"),
                ('topic_estimation', "/estimation/states"),
                ('topic_feasible_path', "/path_planning/feasible_path"),
                ('topic_feasible_waypoints', "/path_planning/feasible_waypoints"),
                ('topic_waypoints', "/path_planning/waypoints/raw"),
                ('topic_obstacles', "/perception/obstacles"),
                ('topic_control_prediction', "/control/prediction"),
                ('topic_control_commands', "/adre_autoboxControl"),
                ('topic_waypoints_rtk', "/path_planning/waypoints/rtk"),
                ('topic_waypoints_rtk_idx', "/path_planning/waypoints/rtk/index"),
                ('topic_autoboxinfo', "/adre_autoboxInfo"),
                ('topic_uwb_switch', "/uwb/rtk_region"),
                ('topic_estimator_variables', "/perception/topic_estimator_variables"),
                ('calibration', False)
            ]
        )

        topic_road_left = self.get_parameter("topic_road_left").get_parameter_value().string_value
        topic_road_right = self.get_parameter("topic_road_right").get_parameter_value().string_value
        topic_fit_left = self.get_parameter("topic_fit_left").get_parameter_value().string_value
        topic_fit_right = self.get_parameter("topic_fit_right").get_parameter_value().string_value
        topic_left_cropped = self.get_parameter("topic_left_cropped").get_parameter_value().string_value
        topic_right_cropped = self.get_parameter("topic_right_cropped").get_parameter_value().string_value
        topic_estimation = self.get_parameter("topic_estimation").get_parameter_value().string_value
        topic_feasible_path = self.get_parameter("topic_feasible_path").get_parameter_value().string_value
        topic_feasible_waypoints = self.get_parameter("topic_feasible_waypoints").get_parameter_value().string_value
        topic_waypoints = self.get_parameter("topic_waypoints").get_parameter_value().string_value
        topic_waypoints_rtk = self.get_parameter("topic_waypoints_rtk").get_parameter_value().string_value
        topic_waypoints_rtk_idx = self.get_parameter("topic_waypoints_rtk_idx").get_parameter_value().string_value
        topic_obstacles = self.get_parameter("topic_obstacles").get_parameter_value().string_value
        topic_control_prediction = self.get_parameter("topic_control_prediction").get_parameter_value().string_value
        topic_control_commands = self.get_parameter("topic_control_commands").get_parameter_value().string_value
        calibration = self.get_parameter("calibration").get_parameter_value().bool_value
        topic_autoboxinfo = self.get_parameter("topic_autoboxinfo").get_parameter_value().string_value
        topic_uwb_switch = self.get_parameter("topic_uwb_switch").get_parameter_value().string_value
        topic_estimator_variables = (self.get_parameter("topic_estimator_variables").get_parameter_value().string_value)
        
        self.total_distance = 0.0
        self.prev_global_state = [0.0]*2

        topic_rtk = self.get_parameter("topic_rtk").get_parameter_value().string_value
        
        self.tdalogger = log.Logger()
        
        self._ego = Vehicle()
        self._ego.add_info({**params["vehicle/kinematics"],**params["vehicle/dynamics"]})
        
        self.spatial_parameters = Parameters(params["path_planning/spatial_filtering"])

        self.projection_percentage = self.spatial_parameters.projection_percentage
        self.projection_length = self.spatial_parameters.projection_length

        self._params_detection = Parameters(params["sensors/detection"])
        self._params_feasible_path = Parameters(params["path_planning/feasible_path"])
        
        self.get_obstacles = self._params_detection.mount
        self.type_obstacles = self._params_detection.type 
        
        if self.get_obstacles:
            if "2d" in self.type_obstacles.lower():
                _ = self.create_subscription(Detections2DArray, topic_obstacles, self.observations_callback, 5)   
            else:
                _ = self.create_subscription(Detections3DArray, topic_obstacles, self.observations_callback, 5)

        # Create Subscription
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        sub_left = Subscriber(self, PointCloud2, topic_road_left)
        sub_right = Subscriber(self, PointCloud2, topic_road_right)
        ats = ApproximateTimeSynchronizer([sub_left, sub_right], queue_size=5, slop=0.2, allow_headerless=True)
        ats.registerCallback(self.road_borders_callback)
        
        sub_fit_left = Subscriber(self, StateArray, topic_fit_left)
        sub_fit_right = Subscriber(self, StateArray, topic_fit_right)
        ats = ApproximateTimeSynchronizer([sub_fit_left, sub_fit_right], queue_size=5, slop=0.2, allow_headerless=True)
        ats.registerCallback(self.fit_callback)


        sub_cropped_left = Subscriber(self, StateArray, topic_left_cropped)
        sub_cropped_right = Subscriber(self, StateArray, topic_right_cropped)
        ats = ApproximateTimeSynchronizer([sub_cropped_left, sub_cropped_right], queue_size=5, slop=0.2, allow_headerless=True)
        ats.registerCallback(self.cropped_callback)

        if calibration:
            topic_estimation += "/rtk"
            self.get_logger().info("CALIBRATION MODE ACTIVE")

        _ = self.create_subscription(State, topic_estimation, self.estimation_callback, 5)
        _ = self.create_subscription(StateArray, topic_feasible_waypoints, self.feasible_waypoints_callback, 5)
        _ = self.create_subscription(StateArray, topic_waypoints, self.waypoints_callback, 5)
        _ = self.create_subscription(StateArray, topic_feasible_path, self.feasible_path_callback, 5)
        _ = self.create_subscription(StateArray, topic_control_prediction, self.control_callback, 5)
        _ = self.create_subscription(AutoboxControl, topic_control_commands, self.cmd_callback, 5)
        _ = self.create_subscription(StateArray, topic_waypoints_rtk, self.waypoints_rtk_callback, 5)
        _ = self.create_subscription(UInt32, topic_waypoints_rtk_idx, self.waypoints_rtk_idx_callback, 5)
        _ = self.create_subscription(Position, topic_rtk, self.rtk_callback, qos_profile=qos_policy)
        _ = self.create_subscription(AutoboxInfo, topic_autoboxinfo, self.autoboxinfo_callback, qos_profile=qos_policy)
        _ = self.create_subscription(Bool, topic_uwb_switch, self.uwb_switch_callback, 1)
        _ = self.create_subscription(Multi2DArray, topic_estimator_variables, self.estimator_varibales_callback, 1)
        self.est_variable_recieved = False
        
        self.autbox_init = False

        self.last_time = time()
        self.last_update = self.last_time

        self.state_global = np.array([0.0]*6)
        self.state_local = [0.0]*6
        self.path_control = [[0.0]*6]*5
        self.feasible_path = [[0.0]*6]*5
        self.path_delay = [[0.0]*6]*5
        self.border_left = [[0.0]*6]*5
        self.border_right = [[0.0]*6]*5
        self.raw_border_left = [[0.0]*6]*5
        self.raw_border_right = [[0.0]*6]*5
        self.cropped_left = [[0.0]*6]*5
        self.cropped_right = [[0.0]*6]*5
        self.data_left_anomaly, self.data_right_anomaly = np.array([[0.0]*6]*5), np.array([[0.0]*6]*5) 
        self.shift_left = [[0.0]*6]*5
        self.shift_right = [[0.0]*6]*5
        self.fit_left = [[0.0]*6]*5
        self.fit_right = [[0.0]*6]*5
        self.cmd = [0.0]*3
        self.observations = list()
        self.feasible_waypoints = [[0.0]*6]*5
        self.waypoints = [[0.0]*6]*5
        self.waypoints_rtk = [[0.0]*6]*5
        self.idx_obs = None
        self.idx_obs_safe = None
        self._ego = None
        self.counter = int()
        self.waypoints_rtk_idx = int()
        self.status_rtk = "Not Active"
        self.fitting_region = self.spatial_parameters.road_fitting_region
        self.uwb_switch = False
        self.focus_dist = [1000.0]
        self.init_time = time()
        self.create_timer(0.1, self.create_visualization_local)


    def estimator_varibales_callback(self, msg):
        """
        Receive data every time that comes new info from /estimation/state topic.
        """
        testing_parameters = msg.data
        meas_pred_states, measurement, curr_states, P_cov, R, E_cov, S, y_res, flag, self.fitting_region, self.focus_dist = testing_parameters[0].data, testing_parameters[1].data,testing_parameters[2].data,testing_parameters[3].data,testing_parameters[4].data, testing_parameters[5].data, testing_parameters[6].data, testing_parameters[7].data, testing_parameters[8].data, testing_parameters[9].data, testing_parameters[10].data
    

    def uwb_switch_callback(self, msg):
        self.uwb_switch = msg.data

    def autoboxinfo_callback(self, msg):

        self.driving_stats = msg.driver
        self.driving_state = msg.state
        self.autbox_init = True
        # print ("state", state , "throttle", driver_stats.throttle, "braking", driver_stats.braking, "steering", driver_stats.steering, "indicator", driver_stats.indicator, "targetVelocity", driver_stats.targetVelocity, "timeGap", driver_stats.timeGap, "button_plus", driver_stats.button_plus, "button_minus", driver_stats.button_minus, "button_event", driver_stats.button_event, "button_enter", driver_stats.button_enter, "button_resume", driver_stats.button_resume)

    def add_initial_border(self, data, width):
        if len(data)>0:    
            per = self.projection_percentage
            no_pts = int(len(data)*per)
            # print ("no_pts", no_pts)
                
            straight_line_x = np.linspace(0, self.projection_length, no_pts)
            straight_line_y = np.ones(no_pts)*width
            straight_line_z = np.zeros(no_pts)*width ##fake

            straight_line = np.vstack([straight_line_x, straight_line_y, straight_line_z]).T
            data = np.concatenate([straight_line, data])
        return data

    def crop_data(self, data, x_min = 3.5, x_max = 10):
        idx_ = data[:,0] > x_min
        data = data[idx_] 
        idx_ = data[:,0] < x_max
        data = data[idx_] 
        return data

    def find_length(self, data):
        x, y = data[:,0], data[:,1]
        dx = x[1:] - x[:-1]
        dy = y[1:] - y[:-1]
        length = np.cumsum(np.sqrt(dx**2 + dy**2))
        length = np.insert(length, 0, x[0])
        return length

    def crop_length(self, data, len_min = 3.5, len_max = 10):
        if len(data)>0:
            length = self.find_length(data)
            idx_min = length > len_min 
            idx_max = length < len_max
            idx_ = idx_min == idx_max
            data = data[idx_] 
        return data

    def dist(self, data1, data2):
        sx = np.sum(data1**2, axis=1, keepdims=True)
        sy = np.sum(data2**2, axis=1, keepdims=True)
        return np.sqrt(-2 * data1.dot(data2.T) + sx + sy.T)

    def remove_close_border_points(self, data_left, data_right, thres = 1.0):
        distance = self.dist(data_left, data_right)
        idx = distance > thres
        left_idx = np.sum(idx, axis = 1) != idx.shape[1]
        right_idx = np.sum(idx, axis = 0) != idx.shape[0]
        data_left = data_left[left_idx]
        data_right = data_right[right_idx]
        return data_left, data_right

    def road_borders_callback(self, msg_left, msg_right):
        """
        Receive data every time that comes new info from /perception/road_borders_left and /perception/road_borders_right topic.
        """

        curr_time = time()

        # self.border_left = msg2pc(msg_left)
        # self.border_right = msg2pc(msg_right)

        self.raw_border_left = msg2pc(msg_left)
        self.raw_border_right = msg2pc(msg_right)
        
        
        # # self.border_left = self.crop_data(self.border_left, x_min = self.spatial_parameters.road_fitting_region[0], x_max = self.spatial_parameters.road_fitting_region[1])
        # # self.border_right = self.crop_data(self.border_right, x_min = self.spatial_parameters.road_fitting_region[0], x_max = self.spatial_parameters.road_fitting_region[1])
        # if self.spatial_parameters.remove_closer_points:
        #     self.data_left_anomaly, self.data_right_anomaly = self.remove_close_border_points(self.raw_border_left[:,:2], self.raw_border_right[:,:2], thres = self.spatial_parameters.remove_closer_points_thersh)

        # self.border_left = self.crop_data(self.raw_border_left, x_min = self.fitting_region[0], x_max = self.fitting_region[1])
        # self.border_right = self.crop_data(self.raw_border_right, x_min = self.fitting_region[0], x_max = self.fitting_region[1])

        # self.border_left = self.crop_length(self.border_left, len_min = self.fitting_region[0], len_max = self.fitting_region[1])
        # self.border_right = self.crop_length(self.border_right, len_min = self.fitting_region[0], len_max = self.fitting_region[1])


        # self.border_left = self.add_initial_border(self.border_left, 2)
        # self.border_right = self.add_initial_border(self.border_right, -2)

        fps = 1 / (curr_time - self.last_time)
        self.last_time = curr_time

        if (curr_time - self.last_update) > 5.:
            self.get_logger().info('Publishing Control Visualization at %.01f Hz' % fps)
            self.last_update = curr_time

    
    def shift_callback(self, msg_left, msg_right):
        """
        Receive data every time that comes new info from /perception/shift/left and /perception/shift/right topic.
        """
        self.shift_left = statearray_to_list(msg_left, format="xy")
        self.shift_right = statearray_to_list(msg_right, format="xy")


    def fit_callback(self, msg_left, msg_right):
        """
        Receive data every time that comes new info from /perception/fit/left and /perception/fit/right topic.
        """
        self.fit_left = statearray_to_list(msg_left, format="xy")
        self.fit_right = statearray_to_list(msg_right, format="xy")


    def cropped_callback(self, msg_left, msg_right):
        """
        Receive data every time that comes new info from /perception/fit/left and /perception/fit/right topic.
        """
        self.cropped_left = statearray_to_list(msg_left, format="xy")
        self.cropped_right = statearray_to_list(msg_right, format="xy")


    def estimation_callback(self, msg):
        """
        Receive data every time that comes new info from /estimation/states topic.
        """            
        self.state_global = state_to_list(state=msg)
        self.state_local = [0., 0., 0.] + self.state_global[3:]
        dx = self.state_global[0] - self.prev_global_state[0] 
        dy = self.state_global[1] - self.prev_global_state[1]
        self.total_distance += np.sqrt(dx**2 + dy**2)
        self.prev_global_state = self.state_global[:2]

    def control_callback(self, msg_pred):

        self.path_control = statearray_to_list(msg_pred, format="xy")        

        if len(self.path_control) < 1:
            self.path_control = None


    def cmd_callback(self, msg_cmd):
        self.cmd = autoboxcontrol_to_list(msg_cmd)
        
    
    def feasible_path_callback(self, msg_feasible_path):

        self.feasible_path = statearray_to_list(msg_feasible_path, format="state")

        if len(self.feasible_path) < 1:
            self.feasible_path = None


    def feasible_waypoints_callback(self, msg_feasible_waypoints):

        self.feasible_waypoints = statearray_to_list(msg_feasible_waypoints, format="xy")

        if len(self.feasible_waypoints) < 1:
            self.feasible_waypoints = None          

    def waypoints_callback(self, msg_waypoints):

        self.counter += 1
        self.waypoints = statearray_to_list(msg_waypoints, format="xy")

        if len(self.waypoints) < 1:
            self.waypoints = None       
            
    def waypoints_rtk_callback(self, msg_waypoints_rtk):
    
        waypoints_rtk = np.asarray(statearray_to_list(msg_waypoints_rtk, format="xyyaw"))

        self.waypoints_rtk = waypoints_rtk
        
        if len(self.waypoints_rtk) < 1:
            self.waypoints_rtk = None
            
    def waypoints_rtk_idx_callback(self, msg):
        
        self.waypoints_rtk_idx = msg.data

    
    def observations_callback(self, msg_obs):
        observations = list()
        for i, obs in enumerate(msg_obs.detections):
            if "2d" in self.type_obstacles.lower():
                obs = Obstacle(detection2d_to_array(obs)[:7], k_safety=self._params_feasible_path.safety_coefficient, vision_range=[self._params_feasible_path.curve_prediction_distance, 10.], logger=self.tdalogger)
            else:
                obs = Obstacle(detection3d_to_array(obs)[:7], k_safety=self._params_feasible_path.safety_coefficient, vision_range=[self._params_feasible_path.curve_prediction_distance, 10.], logger=self.tdalogger)

            observations += [obs]

        self.observations = observations
        
    def rtk_callback(self, msg_rtk):
        
        self.status_rtk = msg_rtk.fix_type

    def create_visualization_local(self):
        
        feasible_path = self.feasible_path
        path_control = self.path_control
        border_cropped_left = self.cropped_left
        border_cropped_right = self.cropped_right
        raw_border_left = self.raw_border_left
        raw_border_right = self.raw_border_right
        data_left_anomaly, data_right_anomaly = self.data_left_anomaly, self.data_right_anomaly 

        shift_left = self.shift_left
        shift_right = self.shift_right
        fit_left = self.fit_left
        fit_right = self.fit_right
        cmd = self.cmd
        obs_list = self.observations
        waypoints = self.waypoints
        feasible_waypoints = self.feasible_waypoints
        waypoints_rtk = self.waypoints_rtk[self.waypoints_rtk_idx-30:self.waypoints_rtk_idx+70]
        
        if len(waypoints_rtk) < 2:
            waypoints_rtk = [[0.0]*6]*5
        else:
            waypoints_rtk = global_to_local(self.state_global, waypoints_rtk, is_object=True)

        steering = cmd[0] * 180/math.pi

        if cmd[1]>0.0:
            ax = cmd[1]
            brake_a = 0.0

        else:
            brake_a = cmd[1]
            ax = 0.0

        speed = self.state_local[3]*3.6

        plt.clf()

        ## Path Planning
        waypoints = [e[:2] for e in waypoints if not np.isnan(waypoints).any()]
        waypoints_rtk = [e[:2] for e in waypoints_rtk if not np.isnan(waypoints_rtk).any()]
        feasible_waypoints = [e[:2] for e in feasible_waypoints if not np.isnan(feasible_waypoints).any()]
        v_ref = feasible_path[1][3]*3.6 if len(feasible_path)>1 else 0.0
        feasible_path = [e[:2] for e in feasible_path]

        plt.plot([x for (x,y) in waypoints], [y for (x,y) in waypoints], "bx", linewidth=2, label = 'Waypoints')
        plt.plot([x for (x,y) in waypoints_rtk], [y for (x,y) in waypoints_rtk], "rx", linewidth=2, label = 'Waypoints RTK')
        plt.plot([x for (x,y) in feasible_waypoints], [y for (x,y) in feasible_waypoints], "ro", linewidth=3, label = 'feasible Waypoints')
        plt.plot([x for (x,y) in feasible_path], [y for (x,y) in feasible_path], "bo", linewidth=15, label = "feasible path")

        ## Control Prediction
        control = [e[:2] for e in path_control]
        plt.plot([x for (x, y) in control], [y for (x, y) in control], 'kx', linewidth=15, label = "Path control")

        ## Perception raw info
        raw_borders_left = [e[:2] for e in raw_border_left]
        raw_borders_right = [e[:2] for e in raw_border_right]

        plt.plot([x for (x, y) in raw_borders_left], [y for (x, y) in raw_borders_left], 'rx')
        plt.plot([x for (x, y) in raw_borders_right], [y for (x, y) in raw_borders_right], 'gx')

        ## Perception considered points for fitting
        borders_left = [e[:2] for e in border_cropped_left]
        borders_right = [e[:2] for e in border_cropped_right]

        plt.scatter([x for (x, y) in borders_left], [y for (x, y) in borders_left], color = 'red', s = 80, facecolor='white')
        plt.scatter([x for (x, y) in borders_right], [y for (x, y) in borders_right], color = 'green',s = 80, facecolor='white')

        ## anomaly 
        if len(data_left_anomaly) > 0:
            plt.scatter(data_left_anomaly[:,0], data_left_anomaly[:,1], color = 'black', s = 100, facecolor='white', marker='*')
        if len(data_right_anomaly) > 0:
            plt.scatter(data_right_anomaly[:,0], data_right_anomaly[:,1], color = 'black',s = 100, facecolor='white', marker='*')

        ## Perception RANSAC
        fit_left = [e[:2] for e in fit_left]
        fit_right = [e[:2] for e in fit_right]

        plt.plot([x for (x, y) in fit_left], [y for (x, y) in fit_left], 'r--')
        plt.plot([x for (x, y) in fit_right], [y for (x, y) in fit_right], 'g--')

        ## Shifted Limits Obstacle Avoidance
        shift_left = [e[:2] for e in shift_left]
        shift_right = [e[:2] for e in shift_right]

        plt.plot([x for (x, y) in shift_left], [y for (x, y) in shift_left], 'r-', linewidth=3)
        plt.plot([x for (x, y) in shift_right], [y for (x, y) in shift_right], 'g-', linewidth=3)

        if self._ego is None:
            ego_length = 3.5
            ego_width = 1.5
            ego_cog = 0.5
            ego_lR = 0.7
            ego_lF = 1.5

        else:
            ego_length = self._ego.length
            ego_width = self._ego.width
            ego_cog = self._ego.CoG["x"]
            ego_lR = self._ego.lr
            ego_lF = self._ego.lf

        ## Ego Vehicle
        left, bottom, width, height = (-ego_length/2 + ego_cog, -ego_width/2, ego_length, ego_width)
        rect=mpatches.Rectangle((left,bottom),width,height, alpha=0.1, facecolor="black")
        plt.gca().add_patch(rect)

        # Rear wheels
        left, bottom, width, height = (-ego_lR, -ego_width/2, 0.6731, 0.24)
        rect=mpatches.Rectangle((left,bottom),width,height, alpha=1, facecolor="black")
        plt.gca().add_patch(rect)

        left, bottom, width, height = (-ego_lR, ego_width/2, 0.6731, -0.24)
        rect=mpatches.Rectangle((left,bottom),width,height, alpha=1, facecolor="black")
        plt.gca().add_patch(rect)

        # Front wheels
        left, bottom, width, height = (ego_lF, -ego_width/2, 0.6731, 0.24)
        rect=mpatches.Rectangle((left,bottom),width,height, alpha=1, facecolor="black", angle=steering)
        plt.gca().add_patch(rect)

        left, bottom, width, height = (ego_lF, ego_width/2, 0.6731, -0.24)
        rect=mpatches.Rectangle((left,bottom),width,height, alpha=1, facecolor="black", angle=steering)
        plt.gca().add_patch(rect)

        ## Observations raw info
        for obs in obs_list:
            left, bottom = (obs.center[0]-(obs.size_x/2*np.cos(obs.yaw)-obs.size_y/2*np.sin(obs.yaw)), obs.center[1]-(obs.size_x/2*np.sin(obs.yaw)+obs.size_y/2*np.cos(obs.yaw)))
            rect=mpatches.Rectangle((left,bottom), obs.size_x, obs.size_y, alpha=0.5, facecolor="blue", angle=obs.yaw*180/math.pi)
            plt.gca().add_patch(rect)
            arrow = mpatches.Arrow(x=obs.center[0], y=obs.center[1], dx=np.cos(obs.yaw), dy=np.sin(obs.yaw), alpha=0.5, facecolor="blue")
            plt.gca().add_patch(arrow)

        y_offset = 2.5
        ## Control Commands and current speed
        plt.text(-2.5, 1.5 + y_offset, "Max Ref Speed = " + str("%.2f" % v_ref) + " km/h", fontsize="xx-large")
        plt.text(-2.5, 1. + y_offset, "Speed = " + str("%.2f" % speed) + " km/h", fontsize="xx-large")
        plt.text(5., 1.5, "RTK Status: {}".format(self.status_rtk), fontsize="xx-large")
        
        if self.autbox_init:
            plt.text(7., 1.5 + y_offset, "Driving Status: {}".format(self.driving_state), fontsize="xx-large")
            plt.text(7., 1.5 + 0.8*y_offset, "UWB Status: {}".format(self.uwb_switch), fontsize="xx-large")
            plt.text(7., 1.5 + 0.6*y_offset, "Focuas dist: " + str("%.2f" % self.focus_dist[0]), fontsize="xx-large")
            
            # plt.text(13.5, 1.5 + y_offset, "Steering Status: {}".format(self.driving_stats.steering), fontsize="xx-large")
            # plt.text(13.5, 1.5 + 0.80*y_offset, "Throttle Status: {}".format(self.driving_stats.throttle), fontsize="xx-large")
            # plt.text(13.5, 1.5 + 0.60*y_offset, "Brake Status: {}".format(self.driving_stats.braking), fontsize="xx-large")        
            # plt.text(13.5, 1.5 + 0.40*y_offset, "Total Distance: " + str("%.2f" % self.total_distance) + " m", fontsize="xx-large")
            # plt.text(13.5, 1.5 + 0.20*y_offset, "Total Time: " + str("%.2f" % (time() - self.init_time)) + " sec", fontsize="xx-large")

        y_offset = -7.0
        plt.text(10, -1.75 + y_offset, "Steer = " + str("%.2f" % steering) + " °" + "\n" + "Throttle = " + str("%.2f" % ax) + " m/s2" + "\n" + "Brake = " + str("%.2f" % brake_a) + " m/s2", fontsize="xx-large", fontweight="semibold", backgroundcolor="whitesmoke")

        plt.axis([-5, 30, -10, 10])
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid()

        if self.plot_legend:
            plt.legend(loc = 'lower left', fontsize=18)
        plt.pause(0.001)


def main():
    """
    Main algorithm for controller MPC.
    Only launched when we don't want CARLA simulator to be active.
    """
    node_name = "visualization_control"
    rclpy.init()
    
    params_reader = ReaderParamsInfo()
    params = params_reader.read_params(["vehicle/kinematics", "vehicle/dynamics", "sensors/detection", "path_planning/feasible_path", "path_planning/spatial_filtering"])

    publisher_node = ROSManagerVisualization(node_name, params)
    publisher_node.get_logger().info("\033[1m{}\033[0m Node Active".format(node_name))
    
    rclpy.spin(publisher_node)
    
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()