import rclpy
from rclpy.node import Node
from tr_messages.msg import SimGroundTruth
from tr_messages.msg import DetWithImg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import numpy as np
import math
from transforms3d.euler import quat2euler
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import tf2_ros as tf2

class TwoPanelEstimation(Node):
    def __init__(self):
        self.gtruth_sub = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth', #  ALREADY IN WORLD FRAME
            self.camera_callback,
            10
        )   

        self.pub_error = self.create_publisher(String, '/estimation/error', 10)

        self.pub_covariance = self.create_publisher(PoseWithCovarianceStamped, '/covariance', 10)

        self.pub_center = self.create_publisher(PoseStamped, '/estimated_center', 10)

        self.pub_original = self.create_publisher(PoseWithCovarianceStamped, '/original', 10)

        self.kf = KalmanFilter(dim_x=6, dim_z=2)

        # State vector: [x, y, vx, vy, ax, ay]
        self.kf.x = np.zeros(6)

        dt = 0.1 
        self.kf.F = np.array([[1., 0., dt, 0., 0.5*dt**2, 0.],
                              [0., 1., 0., dt, 0., 0.5*dt**2],
                              [0., 0., 1., 0., dt, 0.],
                              [0., 0., 0., 1., 0., dt],
                              [0., 0., 0., 0., 1., 0.],
                              [0., 0., 0., 0., 0., 1.]])

        # Measurement function (observe position only)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
        ])

        self.kf.P *= 1000

        self.kf.R = np.diag([0.01, 0.004])

        self.kf.Q = self._compute_process_noise(dt)

        self.last_time = None
        self.initialized = False

        self.ellipse_points = np.array([])

    def get_normal_vector(quaternion):
        roll, pitch, yaw = quat2euler([quaternion.w,quaternion.x, quaternion.y, quaternion.z])
        forward_x = math.cos(yaw) * math.cos(pitch)
        forward_y = math.sin(yaw) * math.cos(pitch)
        forward_z = math.sin(pitch)
        normal_vector = np.array([forward_x, forward_y, forward_z])
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        return normal_vector

    def get_center_triangle(panel_1, panel_2, normal_1, normal_2):
        p1 = [panel_1.pose.position.x, panel_1.pose.position.y]
        p2 = [panel_2.pose.position.x, panel_2.pose.position.y]
        matrix = inv(np.vstack([normal_1, normal_2]).T)
        diff = p2 - p1
        normalmultiple = (matrix @ diff)[0]
        cmid = (p1 + p2) / 2
        rad = np.linalg.norm((p1 - p2)) / 2
        v = p1 + (normalmultiple * normal_1)
        cmidtov = v - cmid
        return cmid + (rad * cmidtov / np.linalg.norm(cmidtov))

