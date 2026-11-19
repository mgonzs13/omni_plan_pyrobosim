#!/usr/bin/env python3
#
# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import csv
import rclpy
from rclpy.node import Node
from datetime import datetime
from omni_plan_pyrobosim_msgs.msg import OrderCompleted


class OrderCompletedSubscriber(Node):
    """Subscribes to order completed topic and logs data to CSV."""

    def __init__(self, csv_filename: str = None):
        super().__init__("order_completed_subscriber")

        # Setup CSV file
        if csv_filename is None:
            csv_filename = f"order_completed.csv"

        self.csv_file = csv_filename
        self.csv_writer = None
        self.csv_file_handle = None
        self.headers_written = False

        self.subscription = self.create_subscription(
            OrderCompleted, "/order_completed", self.listener_callback, 10
        )

        self.get_logger().info(f"Subscribed to /order_completed")
        self.get_logger().info(f"Saving data to: {os.path.abspath(self.csv_file)}")

        self._message_count = 0

    def listener_callback(self, msg):
        """Callback function that processes incoming messages."""
        self.get_logger().debug(
            f"Received order_completed message for order {msg.order_id}"
        )

        # Initialize CSV writer on first message
        if self.csv_file_handle is None:
            self.csv_file_handle = open(self.csv_file, "w", newline="")

        # Timestamp for this message
        msg_timestamp = datetime.now().isoformat()

        # Format item categories and robots as comma-separated strings
        item_categories_str = ",".join(msg.item_categories)
        robots_involved_str = ",".join(msg.robots_involved)

        # Format start and end times
        start_time_str = datetime.fromtimestamp(
            msg.start_time.sec + msg.start_time.nanosec / 1e9
        ).isoformat()
        end_time_str = datetime.fromtimestamp(
            msg.end_time.sec + msg.end_time.nanosec / 1e9
        ).isoformat()

        data = {
            "receive_timestamp": msg_timestamp,
            "order_id": msg.order_id,
            "table": msg.table,
            "group_size": msg.group_size,
            "item_categories": item_categories_str,
            "total_duration_sec": msg.total_duration_sec,
            "start_time": start_time_str,
            "end_time": end_time_str,
            "robots_involved": robots_involved_str,
        }

        # Write headers on first message
        if not self.headers_written:
            self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=data.keys())
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
    # subscriber = OrderCompletedSubscriber(csv_filename="custom_output.csv")
    subscriber = OrderCompletedSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("Shutting down...")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
