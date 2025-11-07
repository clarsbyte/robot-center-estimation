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

        # pos, v, acc
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

        # P: Initial Covariance (High uncertainty to start)
        self.kf.P *= 1000

        # R: Measurement noise covariance (placeholder, updated dynamically)
        self.kf.R = np.diag([0.01, 0.004])

        # Q: Process noise covariance (will be updated dynamically)
        self.kf.Q = self._compute_process_noise(dt)

        self.last_time = None
        self.initialized = False

        self.ellipse_points = np.array([])

    def _compute_process_noise(self, dt):
        """Compute process noise matrix for constant acceleration model"""
        # q is the parameter for the spectral density of the white noise acceleration
        q = self.get_parameter('process_noise').value 
        
        Q = np.zeros((6, 6))
        
        # Process noise for x-axis (position, velocity, acceleration)
        Q_block = np.array([
            [dt**4/4, dt**3/2, dt**2/2],
            [dt**3/2, dt**2, dt],
            [dt**2/2, dt, 1]
        ]) * q
        
        Q[0:3, 0:3] = Q_block
        
        # Process noise for y-axis
        Q[3:6, 3:6] = Q_block
        
        return Q

    
    def get_current_estimate(self):
        """Get the current Kalman filter estimate"""
        if not self.initialized:
            return None

        return {
            'position': self.kf.x[:2].copy(),
            'velocity': self.kf.x[2:4].copy(),
            'acceleration': self.kf.x[4:6].copy(),
            'position_covariance': self.kf.P[:2, :2].copy(),
            'velocity_covariance': self.kf.P[2:4, 2:4].copy(),
            'acceleration_covariance': self.kf.P[4:6, 4:6].copy()
        }

    def update_state_transition(self, dt):
        """Update state transition matrix based on time step"""
        self.kf.F = np.array([[1., 0., dt, 0., 0.5*dt**2, 0.],
                              [0., 1., 0., dt, 0., 0.5*dt**2],
                              [0., 0., 1., 0., dt, 0.],
                              [0., 0., 0., 1., 0., dt],
                              [0., 0., 0., 0., 1., 0.],
                              [0., 0., 0., 0., 0., 1.]])

    def update_process_noise(self, dt):
        """Update process noise based on time step"""
        self.kf.Q = self._compute_process_noise(dt)

    def ellipsoid_parameters(self, covariance_matrix, camera_pos, direction, distance_to_target=1.0):
        # NOTE: This function seems unused in the callback but is kept for utility.
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
        # NOTE: This function seems unused in the callback but is kept for utility.
        xc, zc = center[0], center[1]    
        a, b = semi_major, semi_minor      
        interval = 0.1  
        t = np.arange(0, 2 * np.pi, interval)
        x_on = xc + a * np.cos(t)
        z_on = zc + b * np.sin(t)
        
        return x_on, z_on

    def camera_callback(self, msg):
        self.get_logger().info("Received ground truth message")
        
        
        panel_pose = msg.secondary_robot.armor_panel_poses[1]
        center_pose = msg.secondary_robot.chassis_pose
        
        quaternion = [
            panel_pose.orientation.w,
            panel_pose.orientation.x,
            panel_pose.orientation.y,
            panel_pose.orientation.z,
        ]
        roll, pitch, yaw = quat2euler(quaternion)

        # Calculate the normal vector of the panel in the world frame
        forward_x = math.cos(yaw) * math.cos(pitch)
        forward_y = math.sin(yaw) * math.cos(pitch)
        forward_z = math.sin(pitch)
        normal_vector = np.array([forward_x, forward_y, forward_z])
        normal_vector = normal_vector / np.linalg.norm(normal_vector)

        angle = np.arctan2(normal_vector[1], normal_vector[0])
        
        # Rotation matrix for the XY plane based on normal vector direction
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])
        
        # Define the variance along (major) and perpendicular (minor) to the normal vector
        var_major = 0.006  # Major axis variance (along the normal, more uncertainty)
        var_minor = 0.002  # Minor axis variance (perpendicular to the normal, less uncertainty)
        covariance_local = np.array([
            [var_major, 0],
            [0, var_minor]
        ])

        # Rotate the covariance to align with normal vector -> This is the dynamic R matrix
        covariance_matrix_2d = rotation_matrix @ covariance_local @ rotation_matrix.T
        
        if np.isnan(covariance_matrix_2d).any():
            return

        camera_pos = np.array([
            panel_pose.position.x,
            panel_pose.position.y,
            panel_pose.position.z
        ])
        
        # displacement
        raw_center = np.array([camera_pos[0], camera_pos[1]])
        raw_center -= 0.20 * normal_vector[:2]  

        
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        p.pose.pose.position.x = raw_center[0]
        p.pose.pose.position.y = raw_center[1]
        p.pose.pose.position.z = camera_pos[2] # Keep Z from measurement
        p.pose.pose.orientation = panel_pose.orientation 

        # Create 6x6 covariance from the dynamic 2x2 measurement covariance (R)
        covariance_6d_raw = np.zeros((6, 6))
        covariance_6d_raw[0:2, 0:2] = covariance_matrix_2d
        
        p.pose.covariance = covariance_6d_raw.flatten().tolist()
        self.pub_covariance.publish(p)


        current_time = self.get_clock().now()
        filtered_center_2d = raw_center # Default for first run

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)

            # Update matrices based on time step
            self.update_state_transition(dt)
            self.update_process_noise(dt)

            # Update measurement noise R with the dynamically calculated covariance
            self.kf.R = covariance_matrix_2d.copy()

            self.kf.predict()
            self.kf.update(z=[raw_center[0], raw_center[1]], R=covariance_matrix_2d)

            filtered_center_2d = self.kf.x[:2]

        else:
            # Initialize state vector
            self.kf.x[:2] = raw_center  # position
            self.kf.x[2:4] = 0  # velocity
            self.kf.x[4:6] = 0  # acceleration
            filtered_center_2d = raw_center
            self.initialized = True

        self.last_time = current_time

        # Store results (good practice)
        self.center_results.append({
            'timestamp': current_time,
            'raw_center': raw_center.copy(),
            'filtered_center': filtered_center_2d.copy(),
            'velocity': self.kf.x[2:4].copy() if self.initialized else np.zeros(2),
            'acceleration': self.kf.x[4:6].copy() if self.initialized else np.zeros(2),
            'covariance': self.kf.P.copy()
        })


        center = PoseStamped()
        center.header.frame_id = "map"
        center.header.stamp = self.get_clock().now().to_msg()
        
        center.pose.position.x = float(filtered_center_2d[0])
        center.pose.position.y = float(filtered_center_2d[1])
        center.pose.position.z = float(camera_pos[2])  # Keep original Z

        center.pose.orientation = panel_pose.orientation
        self.pub_center.publish(center)

        p_filtered = PoseWithCovarianceStamped()
        p_filtered.header.stamp = self.get_clock().now().to_msg()
        p_filtered.header.frame_id = 'map'
        p_filtered.pose.pose = center.pose

        pos_covariance = np.zeros((6, 6))
        pos_covariance[0:2, 0:2] = self.kf.P[0:2, 0:2]
        pos_covariance[2, 2] = 1e-6 # Small Z variance
        pos_covariance[3:, 3:] = np.eye(3) * 1e-6 # Small rotation variance
        p_filtered.pose.covariance = pos_covariance.flatten().tolist()
        
        self.pub_covariance.publish(p_filtered) # Publishing the filtered P matrix (overwrites raw R above)


        original_pose = PoseWithCovarianceStamped()
        original_pose.header.frame_id = "map"
        original_pose.header.stamp = self.get_clock().now().to_msg()
        original_pose.pose.pose.position.x = center_pose.position.x
        original_pose.pose.pose.position.y = center_pose.position.y
        original_pose.pose.pose.position.z = center_pose.position.z
        original_pose.pose.pose.orientation = center_pose.orientation
        self.pub_original.publish(original_pose)

        error_x = filtered_center_2d[0] - center_pose.position.x
        error_y = filtered_center_2d[1] - center_pose.position.y
        error = math.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(f"Estimation error: {error:.4f} meters")
        if self.initialized:
            self.get_logger().info(f"Velocity: [{self.kf.x[2]:.4f}, {self.kf.x[3]:.4f}] m/s, "
                                 f"Acceleration: [{self.kf.x[4]:.4f}, {self.kf.x[5]:.4f}] m/s²")
        

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