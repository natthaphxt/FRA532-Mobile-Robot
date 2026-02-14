#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
import numpy as np
import math
import csv
import os


class EKFOdometryNode(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.wheel_radius = 0.033
        self.wheel_base = 0.160
        self.x = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.Q = np.diag([0.001, 0.001, 0.001])
        self.R = np.array([[5.0]])
        self.x_raw = np.zeros(3)

        self.last_time = None
        self.left_joint_idx = None
        self.right_joint_idx = None
        self.joints_found = False

        home_path = os.path.expanduser("~")

        # EKF CSV
        ekf_path = os.path.join(home_path, "ekf_trajectory.csv")
        self.csv_file = open(ekf_path, "w")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "x", "y", "theta"])
        self.csv_file.flush()

        # Raw CSV
        raw_path = os.path.join(home_path, "raw_trajectory.csv")
        self.raw_file = open(raw_path, "w")
        self.raw_writer = csv.writer(self.raw_file)
        self.raw_writer.writerow(["timestamp", "x", "y", "theta"])
        self.raw_file.flush()

        # Publishers
        self.path_pub = self.create_publisher(Path, "/ekf_path", 10)
        self.path_msg = Path()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.get_logger().info(
            "✅ FIXED EKF Node Started - Robust Joint Search Enabled"
        )

    def find_wheel_indices(self, names):
        """Helper to find wheel indices dynamically."""
        left_idx = -1
        right_idx = -1

        left_patterns = ["wheel_left", "left_wheel", "left"]
        right_patterns = ["wheel_right", "right_wheel", "right"]

        for i, name in enumerate(names):
            if any(pat in name for pat in left_patterns):
                left_idx = i
                break

        for i, name in enumerate(names):
            if any(pat in name for pat in right_patterns):
                right_idx = i
                break

        return left_idx, right_idx

    def joint_state_callback(self, msg):
        current_time = Time.from_msg(msg.header.stamp)

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:  
            self.last_time = current_time
            return

        if not self.joints_found:
            l_idx, r_idx = self.find_wheel_indices(msg.name)
            if l_idx != -1 and r_idx != -1:
                self.left_joint_idx, self.right_joint_idx = l_idx, r_idx
                self.joints_found = True

        try:
            v_l = msg.velocity[self.left_joint_idx]
            v_r = msg.velocity[self.right_joint_idx]
        except (IndexError, ValueError):
            return

        v = (v_l + v_r) / 2.0
        w = (v_r - v_l) / self.wheel_base

        theta_old = self.x[2]
        delta_theta = w * dt
        avg_theta = theta_old + (delta_theta / 2.0)

        # Update EKF State (Prediction Step)
        self.x[0] += v * dt * math.cos(avg_theta)
        self.x[1] += v * dt * math.sin(avg_theta)
        self.x[2] = self.normalize_angle(theta_old + delta_theta)

        theta_raw_old = self.x_raw[2]
        self.x_raw[0] += v * dt * math.cos(theta_raw_old + (delta_theta / 2.0))
        self.x_raw[1] += v * dt * math.sin(theta_raw_old + (delta_theta / 2.0))
        self.x_raw[2] = self.normalize_angle(theta_raw_old + delta_theta)

        Fx = np.eye(3)
        Fx[0, 2] = -v * dt * math.sin(avg_theta)
        Fx[1, 2] = v * dt * math.cos(avg_theta)
        self.P = Fx @ self.P @ Fx.T + self.Q

        self.last_time = current_time
        self.publish_odometry(current_time, v, w)

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw_imu = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        z = np.array([yaw_imu])
        H = np.array([[0, 0, 1]])

        y = z - H @ self.x
        y[0] = self.normalize_angle(y[0])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = self.normalize_angle(self.x[2])

        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    def publish_odometry(self, current_time, v, w):
        t_sec = current_time.nanoseconds / 1e9
        self.csv_writer.writerow([t_sec, self.x[0], self.x[1], self.x[2]])
        self.csv_file.flush()

        self.raw_writer.writerow([t_sec, self.x_raw[0], self.x_raw[1], self.x_raw[2]])
        self.raw_file.flush()

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x[0]
        odom.pose.pose.position.y = self.x[1]

        q = self.quaternion_from_euler(0, 0, self.x[2])
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x[0]
        t.transform.translation.y = self.x[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        pose = PoseStamped()
        pose.header.stamp = current_time.to_msg()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.x[0]
        pose.pose.position.y = self.x[1]
        pose.pose.orientation = q

        self.path_msg.header.stamp = current_time.to_msg()
        self.path_msg.header.frame_id = "odom"
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return 0, 0, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def destroy_node(self):
        self.csv_file.close()
        self.raw_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
