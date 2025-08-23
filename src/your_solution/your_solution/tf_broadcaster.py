import rclpy
import rclpy.node
import rclpy.qos
import rclpy.duration

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped, Pose, PoseStamped

from tr_messages.msg import DetWithImg, SimGroundTruth, RobotGroundTruth


class CalcErrorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("calc_error")

        self.detections_sub = self.create_subscription(
            DetWithImg,
            "detections",
            self.det_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.ground_truth_sub = self.create_subscription(
            SimGroundTruth,
            "simulation/ground_truth",
            self.ground_truth_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.latest_ground_truth = SimGroundTruth()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def det_callback(self, msg):
        # need to make this a transform stamped because im lazy and thats what my calc_norm() function needs
        detected_panel_in_cam_frame = TransformStamped()
        p_pose: Pose = msg.detection_info.detections[0].results[0].pose.pose

        detected_panel_in_cam_frame.header.frame_id = "camera_frame"
        detected_panel_in_cam_frame.header.stamp = msg.detection_info.header.stamp
        detected_panel_in_cam_frame.child_frame_id = "detected_panel"
        detected_panel_in_cam_frame.transform.translation.x = p_pose.position.x
        detected_panel_in_cam_frame.transform.translation.y = p_pose.position.y
        detected_panel_in_cam_frame.transform.translation.z = p_pose.position.z
        detected_panel_in_cam_frame.transform.rotation = p_pose.orientation

        self.tf_broadcaster.sendTransform(detected_panel_in_cam_frame)

        panel_ground_truth_in_cam_frame = []

        for i in range(4):
            try:
                target_in_camera_frame = self.tf_buffer.lookup_transform(
                    # careful about order here
                    "camera_frame",
                    f"panel_{i}",
                    detected_panel_in_cam_frame.header.stamp,
                )
            except Exception as e:
                print("EXCEPTION: ", e)
                return

            panel_ground_truth_in_cam_frame.append(target_in_camera_frame)

        # debug print
        # print(-1, " | ", detected_panel_in_cam_frame.transform.translation)

        lowest_panel_error_panel = panel_ground_truth_in_cam_frame[0]
        for i, panel in enumerate(panel_ground_truth_in_cam_frame):
            if self.calc_linear_norm(
                panel, detected_panel_in_cam_frame
            ) < self.calc_linear_norm(
                lowest_panel_error_panel, detected_panel_in_cam_frame
            ):
                lowest_panel_error_panel = panel

            # debug print
            # print(
            #     i,
            #     " | ",
            #     panel.transform.translation,
            #     " | ",
            #     self.calc_linear_norm(panel, detected_panel_in_cam_frame),
            # )

    def calc_linear_norm(self, p1: TransformStamped, p2: TransformStamped):
        norm = (p1.transform.translation.x - p2.transform.translation.x) ** 2
        norm += (p1.transform.translation.y - p2.transform.translation.y) ** 2
        norm += (p1.transform.translation.z - p2.transform.translation.z) ** 2

        norm = norm**0.5
        return norm

    def ground_truth_callback(self, msg):
        self.latest_ground_truth = msg

        camera_pose: Pose = msg.primary_robot.camera_pose

        camera_transform = TransformStamped()
        camera_transform.header = msg.header
        camera_transform.child_frame_id = "camera_frame"

        camera_transform.transform.translation.x = camera_pose.position.x
        camera_transform.transform.translation.y = camera_pose.position.y
        camera_transform.transform.translation.z = camera_pose.position.z
        camera_transform.transform.rotation = camera_pose.orientation

        self.tf_broadcaster.sendTransform(camera_transform)

        # -- broadcast armor panel transforms --

        for i, panel in enumerate(msg.secondary_robot.armor_panel_poses):
            panel: Pose
            panel_transform = TransformStamped()
            panel_transform.header = msg.header
            panel_transform.child_frame_id = f"panel_{i}"

            panel_transform.transform.translation.x = panel.position.x
            panel_transform.transform.translation.y = panel.position.y
            panel_transform.transform.translation.z = panel.position.z
            panel_transform.transform.rotation = panel.orientation

            self.tf_broadcaster.sendTransform(panel_transform)


def main():
    rclpy.init()

    node = CalcErrorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
