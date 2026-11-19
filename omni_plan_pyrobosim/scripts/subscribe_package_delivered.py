#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from datetime import datetime
from omni_plan_pyrobosim_msgs.msg import PackageDelivered


class PackageDeliveredSubscriber(Node):
    """Subscribes to package_delivered topic and logs data to CSV."""

    def __init__(self, csv_filename: str = None):
        super().__init__("package_delivered_subscriber")

        # Setup CSV file
        if csv_filename is None:
            csv_filename = f"package_delivered.csv"

        self.csv_file = csv_filename
        self.csv_writer = None
        self.csv_file_handle = None
        self.headers_written = False

        self.subscription = self.create_subscription(
            PackageDelivered, "/package_delivered", self.listener_callback, 10
        )

        self.get_logger().info(f"Subscribed to /package_delivered")
        self.get_logger().info(f"Saving data to: {os.path.abspath(self.csv_file)}")

        self._message_count = 0

    def listener_callback(self, msg):
        """Callback function that processes incoming messages."""
        self.get_logger().debug(
            f"Received package_delivered message for package {msg.package_id}"
        )

        # Initialize CSV writer on first message
        if self.csv_file_handle is None:
            self.csv_file_handle = open(self.csv_file, "w", newline="")

        # Timestamp for this message
        msg_timestamp = datetime.now().isoformat()

        # Format start and end times
        start_time_str = datetime.fromtimestamp(
            msg.start_time.sec + msg.start_time.nanosec / 1e9
        ).isoformat()
        end_time_str = datetime.fromtimestamp(
            msg.end_time.sec + msg.end_time.nanosec / 1e9
        ).isoformat()

        data = {
            "receive_timestamp": msg_timestamp,
            "package_id": msg.package_id,
            "total_duration_sec": msg.total_duration_sec,
            "start_time": start_time_str,
            "end_time": end_time_str,
        }

        # Write headers on first message
        if not self.headers_written:
            self.csv_writer = csv.DictWriter(
                self.csv_file_handle, fieldnames=data.keys()
            )
            self.csv_writer.writeheader()
            self.headers_written = True

        # Write data row
        self.csv_writer.writerow(data)
        self.csv_file_handle.flush()

        self._message_count += 1
        if self._message_count % 5 == 0:
            self.get_logger().info(f"Received {self._message_count} messages")

    def destroy_node(self):
        """Clean up resources when shutting down."""
        if self.csv_file_handle is not None:
            self.csv_file_handle.close()
            self.get_logger().info(
                f"CSV file closed. Total messages: {self._message_count}"
            )
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    # Optional: specify custom CSV filename
    # subscriber = PackageDeliveredSubscriber(csv_filename="custom_output.csv")
    subscriber = PackageDeliveredSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("Shutting down...")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
