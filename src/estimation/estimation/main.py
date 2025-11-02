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



# NOTE: GET WOLRLD FRAME -> MAP FRAME TRANSFORM
class CenterEstimation(Node):
    def __init__(self):
        super().__init__("center_estimation")

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

        self.center_results = []

        # Kalman Stuff
        self.declare_parameter('process_noise', 0.001)
        self.declare_parameter('measurement_noise', 0.1)
        self.declare_parameter('initial_covariance', 10.0)

        proc_noise = self.get_parameter('process_noise').value
        meas_noise = self.get_parameter('measurement_noise').value
        init_cov = self.get_parameter('initial_covariance').value

        # Kalman filter for center estimation (position and velocity)
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # State vector: [x, y, z, vx, vy, vz]
        self.kf.x = np.zeros(4)

        # State transition matrix (constant velocity model)
        dt = 0.1  # Default time step, will be updated dynamically
        self.kf.F = np.array([[1., 0., dt, 0.],
                 [0., 1., 0., dt],
                 [0., 0., 1., 0.],
                 [0., 0., 0., 1.]])

        # Measurement function (observe position only)
        self.kf.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ])

        # Initial covariance matrix (4x4 for state [x, y, vx, vy])
        self.kf.P = np.eye(4) * init_cov
        # Set position covariance from ellipse
        self.kf.P[0, 0] = 0.006  # X variance
        self.kf.P[1, 1] = 0.002  # Y variance
        self.kf.P[0, 1] = 0        # X-Y correlation
        self.kf.P[1, 0] = 0        # Y-X correlation

        # Measurement noise covariance
        self.kf.R = np.diag([0.006, 0.002])

        # Process noise covariance
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1, block_size=2)

        self.last_time = None
        self.initialized = False

        self.ellipse_points = np.array([])

    
    def get_current_estimate(self):
        """Get the current Kalman filter estimate"""
        if not self.initialized:
            return None

        return {
            'position': self.kf.x[:2].copy(),
            'velocity': self.kf.x[2:4].copy(),
            'position_covariance': self.kf.P[:2, :2].copy(),
            'velocity_covariance': self.kf.P[2:4, 2:4].copy()
        }

    def update_process_noise(self, dt):
        """Update process noise based on time step"""

        q_pos = self.get_parameter('process_noise').value

        # 2D process noise matrix
        self.kf.Q = np.array([
            [q_pos*dt**4/4, 0, q_pos*dt**3/2, 0],
            [0, q_pos*dt**4/4, 0, q_pos*dt**3/2],
            [q_pos*dt**3/2, 0, q_pos*dt**2, 0],
            [0, q_pos*dt**3/2, 0, q_pos*dt**2]
        ])

    def ellipsoid_parameters(self, covariance_matrix, camera_pos, direction, distance_to_target=1.0):
        # Kalman filter input is the center of the ellipsoid
        eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
        sorted_indices = np.argsort(eigenvalues)[::-1]
        sorted_eigenvalues = eigenvalues[sorted_indices]
        sorted_eigenvectors = eigenvectors[:, sorted_indices]

        major_axis = sorted_eigenvectors[:, 0]
        minor_axis = sorted_eigenvectors[:, 1]
        
        center_offset = direction * distance_to_target
        
        #self.get_logger().info(f"Center offset: {center_offset}, Major axis: {major_axis}, Minor axis: {minor_axis}")
        return center_offset, major_axis, minor_axis, sorted_eigenvalues

    def get_ellipse_points(self,center,semi_major, semi_minor):
        xc, zc = center[0], center[1]    
        a, b = semi_major, semi_minor      
        interval = 0.1  
        t = np.arange(0, 2 * np.pi, interval)
        x_on = xc + a * np.cos(t)
        z_on = zc + b * np.sin(t)
        
        return x_on, z_on

    def camera_callback(self, msg):
        self.get_logger().info("Received ground truth message")
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        panel_pose = msg.secondary_robot.armor_panel_poses[1]
        center_pose = msg.secondary_robot.chassis_pose
        
        quaternion = [
            panel_pose.orientation.w,
            panel_pose.orientation.x,
            panel_pose.orientation.y,
            panel_pose.orientation.z,
        ]
        roll, pitch, yaw = quat2euler(quaternion)

        forward_x = math.cos(yaw) * math.cos(pitch)
        forward_y = math.sin(yaw) * math.cos(pitch)
        forward_z = math.sin(pitch)
        normal_vector = np.array([forward_x, forward_y, forward_z])
        normal_vector = normal_vector / np.linalg.norm(normal_vector)

        # The angle we need is the direction of the normal vector projected onto XY plane
        angle = np.arctan2(normal_vector[1], normal_vector[0])
        
        # Rotation matrix for the XY plane based on normal vector direction
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])
        

        var_major = 0.006  # Major axis variance
        var_minor = 0.002  # Minor axis variance
        covariance_local = np.array([
            [var_major, 0],
            [0, var_minor]
        ])

        # Rotate the covariance to align with normal vector
        covariance_matrix_2d = rotation_matrix @ covariance_local @ rotation_matrix.T
        
        if np.isnan(covariance_matrix_2d).any():
            return

        camera_pos = np.array([
            panel_pose.position.x,
            panel_pose.position.y,
            panel_pose.position.z
        ])

        theta = np.linspace(0, 2 * np.pi, 100)
        # Points on ellipse in local frame
        x_local = np.sqrt(var_major) * np.cos(theta)
        y_local = np.sqrt(var_minor) * np.sin(theta)
        
        points_local = np.vstack([x_local, y_local])
        
        points_rotated = rotation_matrix @ points_local
        
        # Translate to camera position
        x_world = points_rotated[0, :] + camera_pos[0]
        y_world = points_rotated[1, :] + camera_pos[1]
        
        #self.get_logger().info(f"Normal vector: {normal_vector}, Angle: {np.degrees(angle):.2f}°")
        
        xy = zip(x_world, y_world)
        result_list = [[x_val, y_val] for x_val, y_val in xy]
        self.ellipse_points = np.append(self.ellipse_points, result_list)

        # Displacement
        raw_center = np.array([camera_pos[0], camera_pos[1]])
        raw_center -= 0.2 * normal_vector[:2]  

        # Fix the axis 
        p.pose.pose.position.x = raw_center[0]
        p.pose.pose.position.y = raw_center[1]
        p.pose.pose.position.z = camera_pos[2]
        p.pose.pose.orientation.x = math.sqrt(2)/2
        p.pose.pose.orientation.y = 0.0
        p.pose.pose.orientation.z = 0.0
        p.pose.pose.orientation.w = math.sqrt(2)/2

        # Create 3D covariance from 2D
        covariance_6d = np.zeros((6, 6))
        covariance_6d[0, 0] = covariance_matrix_2d[0, 0]  # X variance
        covariance_6d[0, 1] = covariance_matrix_2d[0, 1]  # X-Y correlation
        covariance_6d[1, 0] = covariance_matrix_2d[1, 0]  # Y-X correlation
        covariance_6d[1, 1] = covariance_matrix_2d[1, 1]  # Y variance
        covariance_6d[2, 2] = 0  
        covariance_6d[3, 3] = 0
        covariance_6d[4, 4] = 0
        covariance_6d[5, 5] = 0

        p.pose.covariance = covariance_6d.flatten().tolist()

        center = PoseStamped()
        center.header.frame_id = "map"
        center.header.stamp = self.get_clock().now().to_msg()

        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)

            self.kf.F[0, 2] = dt
            self.kf.F[1, 3] = dt

            self.update_process_noise(dt)

            self.kf.R = covariance_matrix_2d.copy()

            self.kf.predict()
            self.kf.update(raw_center)

            filtered_center_2d = self.kf.x[:2]

        else:
            self.kf.x[:2] = raw_center
            self.kf.x[2:] = 0
            filtered_center_2d = raw_center
            self.initialized = True

        self.last_time = current_time

        self.center_results.append({
            'timestamp': current_time,
            'raw_center': raw_center.copy(),
            'filtered_center': filtered_center_2d.copy(),
            'velocity': self.kf.x[2:4].copy() if self.initialized else np.zeros(2),
            'covariance': self.kf.P.copy()
        })

        center.pose.position.x = float(filtered_center_2d[0])
        center.pose.position.y = float(filtered_center_2d[1])
        center.pose.position.z = float(camera_pos[2])  # Keep original Z

        center.pose.orientation.x = panel_pose.orientation.x
        center.pose.orientation.y = panel_pose.orientation.y
        center.pose.orientation.z = panel_pose.orientation.z
        center.pose.orientation.w = panel_pose.orientation.w

        p_filtered = PoseWithCovarianceStamped()
        p_filtered.header.stamp = self.get_clock().now().to_msg()
        p_filtered.header.frame_id = 'map'
        p_filtered.pose.pose = center.pose

        pos_covariance = np.zeros((6, 6))
        # Map 2D covariance to 3D (X, Y positions)
        pos_covariance[0, 0] = self.kf.P[0, 0]  # X variance
        pos_covariance[1, 1] = self.kf.P[1, 1]  # Y variance
        pos_covariance[0, 1] = self.kf.P[0, 1]  # X-Y correlation
        pos_covariance[1, 0] = self.kf.P[1, 0]  # Y-X correlation
        pos_covariance[2, 2] = 0.0  # Small Z variance
        pos_covariance[3:, 3:] = np.eye(3) * 0.0  # Rotation variance
        p_filtered.pose.covariance = pos_covariance.flatten().tolist()

        original_pose = PoseWithCovarianceStamped()
        original_pose.header.frame_id = "map"
        original_pose.header.stamp = self.get_clock().now().to_msg()
        original_pose.pose.pose.position.x = center_pose.position.x
        original_pose.pose.pose.position.y = center_pose.position.y
        original_pose.pose.pose.position.z = center_pose.position.z
        original_pose.pose.pose.orientation = center_pose.orientation

        error_x = filtered_center_2d[0] - center_pose.position.x
        error_y = filtered_center_2d[1] - center_pose.position.y
        error = math.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(f"Estimation error: {error:.4f} meters")

# get panel, get translation, ellipsoid, do translation
        self.pub_covariance.publish(p)
        self.pub_center.publish(center)
        self.pub_original.publish(original_pose)

        #if len(self.center_results) % 10 == 0:  # Log every 10 measurements
        #    self.get_logger().info(f"Filtered: {filtered_center_2d}, Velocity: {self.kf.x[2:4]}")
# GET KALMAN FILTER TO ESTIMATE THE CENTER CAUSE THE ELLIPOSID SHOULD MASK THE CENTER

def main():
    rclpy.init()
    node = CenterEstimation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()