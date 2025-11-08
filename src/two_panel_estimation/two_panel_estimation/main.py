import rclpy
from rclpy.node import Node
from tr_messages.msg import SimGroundTruth
from tr_messages.msg import DetWithImg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import numpy as np
from numpy.linalg import inv
import math
from transforms3d.euler import quat2euler
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import tf2_ros as tf2

# TODO: ADD BIAS TO PRIORITIZE RECENT MEASUREMENT
class TwoPanelEstimation(Node):
    def __init__(self):
        super().__init__('two_panel_estimation')
        self.gtruth_sub = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth', #  ALREADY IN WORLD FRAME
            self.camera_callback,
            10
        )   
        
        self.pub_panel_1 = self.create_publisher(PoseStamped, '/panel_1', 10)
        self.pub_panel_2 = self.create_publisher(PoseStamped, '/panel_2', 10)

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

        self.fading_factor = 1.05
        self.kf.P *= 1000

        self.kf.R = np.diag([0.005, 0.005])

        self.kf.Q = self._compute_process_noise(dt)

        self.last_time = None
        self.initialized = False

        self.ellipse_points = np.array([])

    def _compute_process_noise(self, dt):
        """Compute process noise covariance matrix Q"""
        q = 0.1  # Process noise magnitude
        return Q_discrete_white_noise(dim=2, dt=dt, var=q**2, block_size=3)

    def update_state_transition(self, dt):
        """Update the state transition matrix with new dt"""
        self.kf.F = np.array([[1., 0., dt, 0., 0.5*dt**2, 0.],
                              [0., 1., 0., dt, 0., 0.5*dt**2],
                              [0., 0., 1., 0., dt, 0.],
                              [0., 0., 0., 1., 0., dt],
                              [0., 0., 0., 0., 1., 0.],
                              [0., 0., 0., 0., 0., 1.]])

    def update_process_noise(self, dt):
        """Update the process noise covariance matrix with new dt"""
        self.kf.Q = self._compute_process_noise(dt)

    @staticmethod
    def get_normal_vector(quaternion):
        roll, pitch, yaw = quat2euler([quaternion.w,quaternion.x, quaternion.y, quaternion.z])
        forward_x = math.cos(yaw) * math.cos(pitch)
        forward_y = math.sin(yaw) * math.cos(pitch)
        forward_z = math.sin(pitch)
        normal_vector = np.array([forward_x, forward_y, forward_z])
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        return normal_vector

    @staticmethod
    def get_center_triangle(panel_1, panel_2, normal_1, normal_2):
        p1 = np.array([panel_1.position.x, panel_1.position.y])
        p2 = np.array([panel_2.position.x, panel_2.position.y])
        # Use only x,y components of the normal vectors for 2D calculation
        normal_1_2d = normal_1[:2]
        normal_2_2d = normal_2[:2]
        matrix = inv(np.vstack([normal_1_2d, normal_2_2d]).T)
        diff = p2 - p1
        normalmultiple = (matrix @ diff)[0]
        cmid = (p1 + p2) / 2
        rad = np.linalg.norm((p1 - p2)) / 2
        v = p1 + (normalmultiple * normal_1_2d)
        cmidtov = v - cmid
        return cmid + (rad * cmidtov / np.linalg.norm(cmidtov))
    
    def return_panel_pose(self, panel):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.position.x = panel.position.x
        p.pose.position.y = panel.position.y
        p.pose.position.z = panel.position.z
        p.pose.orientation = panel.orientation
        return p

    def camera_callback(self, msg):
        current_time = self.get_clock().now()
        panel_pose_1 = msg.secondary_robot.armor_panel_poses[1]
        panel_pose_2 = msg.secondary_robot.armor_panel_poses[2]

        center_pose = msg.secondary_robot.chassis_pose

        normal_1 = self.get_normal_vector(panel_pose_1.orientation)
        normal_2 = self.get_normal_vector(panel_pose_2.orientation)

        cm = self.get_center_triangle(panel_pose_1, panel_pose_2, normal_1, normal_2)
        # WHEN RECTANGULAR MAKE IT ELLPISE WITH THE SEMI MAJOR AXIS ORIENTATION IN THE DIRECTION OF THE LONGER

        covariance = np.array([
            [0.005, 0],
            [0, 0.005]
        ])

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)

            self.update_state_transition(dt)
            self.update_process_noise(dt)

            self.kf.R = covariance.copy()
            

            self.kf.predict()

            self.kf.P *= self.fading_factor
            
            self.kf.update(z=[cm[0], cm[1]], R=covariance)

            filtered_center_2d = self.kf.x[:2]

        else:
            # Initialize state vector
            self.kf.x[:2] = cm  # position
            self.kf.x[2:4] = 0  # velocity
            self.kf.x[4:6] = 0  # acceleration
            filtered_center_2d = cm
            self.initialized = True

        self.last_time = current_time
        p1 = self.return_panel_pose(panel_pose_1) # PANEL 1 PLOT
        p2 = self.return_panel_pose(panel_pose_2) # PANEL 2 PLOT

        # Convert actual center to PoseWithCovarianceStamped
        actual_center = PoseWithCovarianceStamped()
        actual_center.header.stamp = self.get_clock().now().to_msg()
        actual_center.header.frame_id = 'map'
        actual_center.pose.pose.position.x = center_pose.position.x
        actual_center.pose.pose.position.y = center_pose.position.y
        actual_center.pose.pose.position.z = center_pose.position.z
        actual_center.pose.pose.orientation = center_pose.orientation
        actual_center.pose.covariance = np.zeros(36).tolist()  # Zero covariance for ground truth

        estimated_center = PoseStamped()
        estimated_center.header.stamp = self.get_clock().now().to_msg()
        estimated_center.header.frame_id = 'map'
        estimated_center.pose.position.x = filtered_center_2d[0]
        estimated_center.pose.position.y = filtered_center_2d[1]
        estimated_center.pose.position.z = panel_pose_1.position.z # Keep Z from measurement
        estimated_center.pose.orientation = panel_pose_1.orientation
        pos_covariance = np.zeros((6, 6))
        pos_covariance[0:2, 0:2] = covariance

        triangle_center = PoseWithCovarianceStamped()
        triangle_center.header.stamp = self.get_clock().now().to_msg()
        triangle_center.header.frame_id = 'map'
        triangle_center.pose.pose.position.x = cm[0]
        triangle_center.pose.pose.position.y = cm[1]
        triangle_center.pose.pose.position.z = panel_pose_1.position.z # Keep Z from measurement
        triangle_center.pose.pose.orientation = panel_pose_1.orientation
        triangle_center.pose.covariance = pos_covariance.flatten().tolist()

        error = np.linalg.norm(np.array([
            estimated_center.pose.position.x - actual_center.pose.pose.position.x,
            estimated_center.pose.position.y - actual_center.pose.pose.position.y
        ]))
        
        self.get_logger().info(f'Error: {error}')

        self.pub_original.publish(actual_center)
        self.pub_center.publish(estimated_center)
        self.pub_covariance.publish(triangle_center)
        self.pub_panel_1.publish(p1)
        self.pub_panel_2.publish(p2)

        # I NEED TO PLOT THE ACTUAL CENTER AND I NEED TO PLOT THE ESTIMATED CENTER


def main(args=None):
    rclpy.init(args=args)
    node = TwoPanelEstimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()