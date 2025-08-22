import rclpy
import rclpy.node
import rclpy.qos

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
        detected_panel_in_cam_frame = PoseStamped()
        detected_panel_in_cam_frame.pose = (
            msg.detection_info.detections[0].results[0].pose.pose
        )

        camera_pose: Pose = self.latest_ground_truth.primary_robot.camera_pose

        camera_transform = TransformStamped()
        camera_transform.header.frame_id = "map"
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.child_frame_id = "camera_frame"

        camera_transform.transform.translation.x = camera_pose.position.x
        camera_transform.transform.translation.y = camera_pose.position.y
        camera_transform.transform.translation.z = camera_pose.position.z
        camera_transform.transform.rotation = camera_pose.orientation

        self.tf_broadcaster.sendTransform(camera_transform)

        ground_truth_panels: list[Pose] = (
            self.latest_ground_truth.secondary_robot.armor_panel_poses
        )

        panel_ground_truth_in_cam_frame = []

        for i in range(4):
            target_pose = PoseStamped()
            target_pose.pose = ground_truth_panels[i]
            target_pose.header.frame_id = "map"

            try:
                target_in_camera_frame = self.tf_buffer.transform(
                    object_stamped=target_pose,
                    target_frame="camera_frame",
                )
            except Exception as e:
                print("EXCEPTION: ", e)
                return

            panel_ground_truth_in_cam_frame.append(target_in_camera_frame)

        lowest_panel_error_panel = panel_ground_truth_in_cam_frame[0]
        index = 0
        print(-1, " | ", detected_panel_in_cam_frame.pose.position)

        for panel in panel_ground_truth_in_cam_frame:
            if self.calc_linear_norm(
                panel, detected_panel_in_cam_frame
            ) < self.calc_linear_norm(
                lowest_panel_error_panel, detected_panel_in_cam_frame
            ):
                lowest_panel_error_panel = panel

            print(
                index,
                " | ",
                panel.pose.position,
                " | ",
                self.calc_linear_norm(panel, detected_panel_in_cam_frame),
            )
            index += 1
        print("----")
        # print("true_pose", lowest_panel_error_panel.pose.position)
        # print(
        #     "error",
        #     self.calc_linear_norm(
        #         lowest_panel_error_panel, detected_panel_in_cam_frame
        #     ),
        # )

    def calc_linear_norm(self, p1, p2):
        norm = (p1.pose.position.x - p2.pose.position.x) ** 2
        norm += (p1.pose.position.y - p2.pose.position.y) ** 2
        norm += (p1.pose.position.z - p2.pose.position.z) ** 2

        norm = norm**0.5
        return norm

    def ground_truth_callback(self, msg):
        self.latest_ground_truth = msg
        # print("ground_truth", self.latest_ground_truth)


def main():
    rclpy.init()

    node = CalcErrorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
