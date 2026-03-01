#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
from datetime import datetime
import math


class PathLogger(Node):
    def __init__(self):
        super().__init__("path_logger")

        # Parameters
        self.declare_parameter("output_file", "slam_path.csv")
        self.declare_parameter("log_rate", 10.0)

        output_file = self.get_parameter("output_file").value
        log_rate = self.get_parameter("log_rate").value

        # Add timestamp to filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = output_file.replace(".csv", "")
        self.output_file = f"{base_name}_{timestamp}.csv"

        # Create CSV file with headers
        self.csv_file = open(self.output_file, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw"]
        )
        self.csv_file.flush()  # Force write headers immediately

        # TF2 for getting SLAM pose from map->base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to query TF at regular intervals
        self.timer = self.create_timer(1.0 / log_rate, self.timer_callback)

        # Also subscribe to odometry for comparison
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Create separate CSV for odometry
        self.odom_file = f"odom_{timestamp}.csv"
        self.odom_csv = open(self.odom_file, "w", newline="")
        self.odom_writer = csv.writer(self.odom_csv)
        self.odom_writer.writerow(
            ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw"]
        )
        self.odom_csv.flush()

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📝 SLAM path logger started")
        self.get_logger().info(f"   Output: {self.output_file}")
        self.get_logger().info(f"   Odom:   {self.odom_file}")
        self.get_logger().info(f"   Rate:   {log_rate} Hz")
        self.get_logger().info("=" * 60)

        self.pose_count = 0
        self.odom_count = 0
        self.warned = False

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        """Get SLAM pose from TF tree (map -> base_link)"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            # Extract timestamp (use current time)
            now = self.get_clock().now()
            timestamp = (
                now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Extract orientation
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

            # Write to CSV
            self.csv_writer.writerow(
                [timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw]
            )

            # Flush every 10 samples to ensure data is written
            if self.pose_count % 10 == 0:
                self.csv_file.flush()

            self.pose_count += 1
            if self.pose_count % 100 == 0:
                self.get_logger().info(
                    f"✅ Logged {self.pose_count} SLAM poses (x={x:.2f}, y={y:.2f})"
                )

            self.warned = False  # Reset warning flag

        except TransformException as ex:
            if not self.warned:
                self.get_logger().warn(
                    f"⚠️  Waiting for map->base_link transform... ({ex})"
                )
                self.warned = True
            return

    def odom_callback(self, msg):
        """Log odometry to CSV for comparison"""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        self.odom_writer.writerow(
            [timestamp, x, y, z, qx, qy, qz, qw, roll, pitch, yaw]
        )

        # Flush every 10 samples
        if self.odom_count % 10 == 0:
            self.odom_csv.flush()

        self.odom_count += 1

    def destroy_node(self):
        """Close CSV files on shutdown"""
        try:
            self.csv_file.flush()
            self.odom_csv.flush()
            self.csv_file.close()
            self.odom_csv.close()
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"✅ Saved {self.pose_count} SLAM poses to {self.output_file}"
            )
            self.get_logger().info(
                f"✅ Saved {self.odom_count} odom poses to {self.odom_file}"
            )
            self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().error(f"Error closing files: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        self.get_logger().info("Shutting down path logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
