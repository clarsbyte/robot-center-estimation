import rclpy
import rclpy.node
import rclpy.duration

import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64


class CalcError(rclpy.node.Node):
    def __init__(self):
        super().__init__("calc_error")

        self.timer = self.create_timer(0.0067, self.timer_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.x_err_pub = self.create_publisher(Float64, "your_solution/x_err", 10)
        self.y_err_pub = self.create_publisher(Float64, "your_solution/y_err", 10)
        self.z_err_pub = self.create_publisher(Float64, "your_solution/z_err", 10)

    def timer_callback(self):
        transform_time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)

        try:
            detected_panel_in_cam_frame = self.tf_buffer.lookup_transform(
                "camera_frame", "detected_panel", transform_time
            )
        except Exception as e:
            print("EXCEPTION", e)
            return

        panel_ground_truth_in_cam_frame = []

        for i in range(4):
            try:
                target_in_camera_frame = self.tf_buffer.lookup_transform(
                    # careful about order here
                    "camera_frame",
                    f"panel_{i}",
                    transform_time,
                )
            except Exception as e:
                print("EXCEPTION: ", e)
                return

            panel_ground_truth_in_cam_frame.append(target_in_camera_frame)

        # debug print
        # print("detected:", detected_panel_in_cam_frame.transform.translation)

        lowest_panel_error_panel: TransformStamped = panel_ground_truth_in_cam_frame[0]
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
        x_err = Float64()
        x_err.data = (
            detected_panel_in_cam_frame.transform.translation.x
            - lowest_panel_error_panel.transform.translation.x
        )
        y_err = Float64()
        y_err.data = (
            detected_panel_in_cam_frame.transform.translation.y
            - lowest_panel_error_panel.transform.translation.y
        )
        z_err = Float64()
        z_err.data = (
            detected_panel_in_cam_frame.transform.translation.z
            - lowest_panel_error_panel.transform.translation.z
        )
        self.x_err_pub.publish(x_err)
        self.y_err_pub.publish(y_err)
        self.z_err_pub.publish(z_err)

    def calc_linear_norm(self, p1: TransformStamped, p2: TransformStamped):
        norm = (p1.transform.translation.x - p2.transform.translation.x) ** 2
        norm += (p1.transform.translation.y - p2.transform.translation.y) ** 2
        norm += (p1.transform.translation.z - p2.transform.translation.z) ** 2

        norm = norm**0.5
        return norm


def main():
    rclpy.init()
    node = CalcError()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
