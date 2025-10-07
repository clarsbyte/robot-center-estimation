import rclpy
from rclpy.node import Node
from tr_messages.msg import SimGroundTruth
from tr_messages.msg import DetWithImg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import String
import numpy as np

class CenterEstimation(Node):
    x = np.array([])
    y = np.array([])
    z = np.array([])

    def __init__(self):
        super().__init__("center_estimation")

        self.gtruth_sub = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth',
            self.camera_callback,
            10
        )

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/covariance', 10)

    def camera_callback(self, msg):
        p = PoseWithCovarianceStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        panel_pose = msg.secondary_robot.armor_panel_poses[0]
        self.get_logger().info(f"Camera Position: x={panel_pose.position.x}, y={panel_pose.position.y}, z={panel_pose.position.z}")

        self.x = np.append(self.x, panel_pose.position.x)
        self.y = np.append(self.y, panel_pose.position.y)
        self.z = np.append(self.z, panel_pose.position.z)

        p.pose.pose = panel_pose

        data_multi_variable = np.array([self.x, self.y]) 
        covariance_matrix = np.cov(data_multi_variable)

        covariance_6d = np.zeros((6, 6))
        covariance_6d[0:2, 0:2] = covariance_matrix 
        covariance_6d[2, 2] = 0.01  
        covariance_6d[3, 3] = 0.01  
        covariance_6d[4, 4] = 0.01  
        covariance_6d[5, 5] = 0.01  
        self.get_logger().info(f"Covariance Matrix:\n{covariance_6d}")

        p.pose.covariance = covariance_6d.flatten().tolist()

        self.pub.publish(p)


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