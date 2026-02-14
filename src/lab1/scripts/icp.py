#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import numpy as np
from tf2_ros import TransformBroadcaster
import os
from scipy.spatial import KDTree
from collections import deque
import math


class ICPOdometry(Node):
    def __init__(self):
        super().__init__("icp_odometry")

        self.local_map_size = 30  
        self.local_map_voxel_size = 0.03  # ละเอียดขึ้น
        self.keyframe_dist_thresh = 0.1  # ทำ ICP บ่อยขึ้น 
        self.keyframe_angle_thresh = np.radians(5.0)  # ทำ ICP เมื่อหมุนทุก 5 องศา

        self.icp_max_iterations = 50  # เพิ่มรอบการคำนวณ
        self.icp_tolerance = 1e-6
        self.icp_max_correspondence_distance = 0.3  # แคบลงเพื่อความแม่นยำ
        self.min_correspondences = 50  # ต้องการจุดยืนยันมากขึ้น
        self.max_scan_points = 400

        self.max_translation_correction = 0.15
        self.max_rotation_correction = np.radians(5.0)
        # ------------------------

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.local_map_scans = deque(maxlen=self.local_map_size)
        self.local_map_points = None
        self.local_map_tree = None
        self.local_map_dirty = True

        self.last_kf_x, self.last_kf_y, self.last_kf_theta = 0.0, 0.0, 0.0
        self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_theta = None, None, None
        self.latest_ekf_odom = None
        self.ekf_received = False

        self.trajectory = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_policy
        )
        self.ekf_odom_sub = self.create_subscription(
            Odometry, "/odom", self.ekf_odom_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom_icp", 10)
        self.path_pub = self.create_publisher(Path, "/path_icp", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("✅ ICP Node Tuned & Started")

    def ekf_odom_callback(self, msg):
        self.latest_ekf_odom = msg
        self.ekf_received = True

    def scan_callback(self, msg):
        if not self.ekf_received or self.latest_ekf_odom is None:
            return

        current_points = self.scan_to_pointcloud(msg)
        if len(current_points) < self.min_correspondences:
            return

        ekf_x, ekf_y, ekf_theta = self.extract_pose(self.latest_ekf_odom)

        if self.prev_ekf_x is None:
            self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_theta = (
                ekf_x,
                ekf_y,
                ekf_theta,
            )
            self.x, self.y, self.theta = ekf_x, ekf_y, ekf_theta
            self.add_keyframe(current_points)
            return

        # Motion Prediction from EKF
        self.x += ekf_x - self.prev_ekf_x
        self.y += ekf_y - self.prev_ekf_y
        self.theta = self.normalize_angle(
            self.theta + (ekf_theta - self.prev_ekf_theta)
        )

        self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_theta = ekf_x, ekf_y, ekf_theta

        # Trigger ICP
        dist_sq = (self.x - self.last_kf_x) ** 2 + (self.y - self.last_kf_y) ** 2
        angle_diff = abs(self.normalize_angle(self.theta - self.last_kf_theta))

        if (
            dist_sq > self.keyframe_dist_thresh**2
            or angle_diff > self.keyframe_angle_thresh
        ):
            if self.local_map_dirty:
                self.rebuild_local_map()

            if self.local_map_tree is not None:
                pts_icp = current_points
                if len(current_points) > self.max_scan_points:
                    idx = np.random.choice(
                        len(current_points), self.max_scan_points, replace=False
                    )
                    pts_icp = current_points[idx]

                try:
                    refined_x, refined_y, refined_theta, success = self.icp_scan_to_map(
                        pts_icp, self.x, self.y, self.theta
                    )
                    if success:
                        dx_corr = math.hypot(refined_x - self.x, refined_y - self.y)
                        dth_corr = abs(self.normalize_angle(refined_theta - self.theta))

                        if (
                            dx_corr < self.max_translation_correction
                            and dth_corr < self.max_rotation_correction
                        ):
                            self.x, self.y, self.theta = (
                                refined_x,
                                refined_y,
                                refined_theta,
                            )
                            self.add_keyframe(current_points)
                except Exception as e:
                    self.get_logger().error(f"ICP Error: {e}")

        self.publish_odom_and_path(Time.from_msg(msg.header.stamp))

    def add_keyframe(self, points):
        self.local_map_scans.append(
            self.transform_to_odom(points, self.x, self.y, self.theta)
        )
        self.local_map_dirty = True
        self.last_kf_x, self.last_kf_y, self.last_kf_theta = self.x, self.y, self.theta

    def transform_to_odom(self, points, x, y, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.column_stack(
            [
                points[:, 0] * c - points[:, 1] * s + x,
                points[:, 0] * s + points[:, 1] * c + y,
            ]
        )

    def rebuild_local_map(self):
        if not self.local_map_scans:
            return
        all_points = np.vstack(list(self.local_map_scans))
        if self.local_map_voxel_size > 0:
            voxel_indices = np.floor(all_points / self.local_map_voxel_size).astype(int)
            _, idx = np.unique(voxel_indices, axis=0, return_index=True)
            all_points = all_points[idx]
        self.local_map_points = all_points
        self.local_map_tree = KDTree(all_points)
        self.local_map_dirty = False

    def icp_scan_to_map(self, scan_points, init_x, init_y, init_theta):
        x, y, theta = init_x, init_y, init_theta
        prev_error = float("inf")
        for _ in range(self.icp_max_iterations):
            c, s = np.cos(theta), np.sin(theta)
            transformed = np.column_stack(
                [
                    scan_points[:, 0] * c - scan_points[:, 1] * s + x,
                    scan_points[:, 0] * s + scan_points[:, 1] * c + y,
                ]
            )
            dists, indices = self.local_map_tree.query(transformed)
            valid = dists < self.icp_max_correspondence_distance
            if np.sum(valid) < self.min_correspondences:
                return init_x, init_y, init_theta, False

            # Trimmed ICP
            trim_thresh = np.percentile(dists[valid], 80)
            valid &= dists <= trim_thresh

            mean_error = np.mean(dists[valid])
            if abs(prev_error - mean_error) < self.icp_tolerance:
                break
            prev_error = mean_error

            src_m, tgt_m = scan_points[valid], self.local_map_points[indices[valid]]
            src_c, tgt_c = np.mean(src_m, axis=0), np.mean(tgt_m, axis=0)
            W = (tgt_m - tgt_c).T @ (src_m - src_c)
            U, _, Vt = np.linalg.svd(W)
            R = U @ Vt
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = U @ Vt
            theta = np.arctan2(R[1, 0], R[0, 0])
            t = tgt_c - R @ src_c
            x, y = t[0], t[1]
        return x, y, theta, True

    def scan_to_pointcloud(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(len(ranges)) * scan_msg.angle_increment + scan_msg.angle_min
        valid = (
            (ranges > scan_msg.range_min)
            & (ranges < scan_msg.range_max)
            & np.isfinite(ranges)
        )
        return np.column_stack(
            [
                ranges[valid] * np.cos(angles[valid]),
                ranges[valid] * np.sin(angles[valid]),
            ]
        )

    def extract_pose(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def publish_odom_and_path(self, current_time):
        q = Quaternion(
            x=0.0, y=0.0, z=math.sin(self.theta * 0.5), w=math.cos(self.theta * 0.5)
        )
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = current_time.to_msg(), "odom"
        (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.orientation,
        ) = (self.x, self.y, q)
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header, pose.pose = odom.header, odom.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        self.trajectory.append(
            [current_time.nanoseconds / 1e9, self.x, self.y, self.theta]
        )

    def save_trajectory(self):
        file_path = os.path.expanduser("~/icp_trajectory.csv")
        np.savetxt(
            file_path,
            np.array(self.trajectory),
            fmt="%.6f",
            delimiter=",",
            header="timestamp,x,y,theta",
            comments="",
        )
        self.get_logger().info(f"💾 Trajectory saved to {file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = ICPOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_trajectory()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
