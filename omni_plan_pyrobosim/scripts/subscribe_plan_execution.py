#!/usr/bin/env python3

import os
import csv
import rclpy
from rclpy.node import Node
from datetime import datetime
from omni_plan_msgs.msg import PlanExecutionStatus


class PlanExecutionSubscriber(Node):
    """Subscribes to plan execution topic and logs data to CSV."""

    # Status code mappings
    STATUS_NAMES = {
        # 0: "PENDING",
        # 1: "RUNNING",
        2: "SUCCEEDED",
        3: "FAILED",
        4: "CANCELLED",
        5: "SKIPPED",
    }

    def __init__(self, csv_filename: str = None):
        super().__init__("plan_execution_subscriber")

        # Setup CSV file
        if csv_filename is None:
            csv_filename = f"plan_execution.csv"

        self.csv_file = csv_filename
        self.csv_writer = None
        self.csv_file_handle = None
        self.headers_written = False

        self.subscription = self.create_subscription(
            PlanExecutionStatus, "/omni_plan/plan_execution", self.listener_callback, 10
        )

        self.get_logger().info(f"Subscribed to /omni_plan/plan_execution")
        self.get_logger().info(f"Saving data to: {os.path.abspath(self.csv_file)}")

        self._message_count = 0
        # Track seen actions to avoid duplicates: {(action_name, node_id, plan_id): status}
        self._seen_actions = {}
        # Track overall status to detect new plans
        self._last_overall_status = None
        self._plan_counter = 0

    def listener_callback(self, msg):
        """Callback function that processes incoming messages."""
        self.get_logger().debug(f"Received message with {len(msg.actions)} actions")

        # Initialize CSV writer on first message
        if self.csv_file_handle is None:
            self.csv_file_handle = open(self.csv_file, "w", newline="")

        # Timestamp for this message
        msg_timestamp = datetime.now().isoformat()

        # Status name mapping
        overall_status_name = self._get_overall_status_name(msg.overall_status)

        # Detect new plan: when overall status transitions to RUNNING from IDLE/terminal states
        if self._last_overall_status is not None:
            # New plan detected when transitioning to RUNNING from non-running state
            if (
                msg.overall_status == 1 and self._last_overall_status != 1
            ):  # 1 = RUNNING
                self._plan_counter += 1
                self._seen_actions.clear()
                self.get_logger().info(
                    f"New plan detected (plan #{self._plan_counter})"
                )

        self._last_overall_status = msg.overall_status

        # Write each action to CSV
        for action in msg.actions:
            status_name = self.STATUS_NAMES.get(action.status, None)

            if status_name is None:
                continue

            # Key includes plan counter to distinguish actions from different plans
            action_key = (action.action_name, action.node_id, self._plan_counter)

            # Only write if this is a new action or status has changed
            if (
                action_key not in self._seen_actions
                or self._seen_actions[action_key] != status_name
            ):
                data = {
                    "timestamp": msg_timestamp,
                    "plan_id": self._plan_counter,
                    "overall_status": overall_status_name,
                    "action_name": action.action_name,
                    "status": status_name,
                    "parameters": (
                        ",".join(action.parameters) if action.parameters else ""
                    ),
                    "node_id": action.node_id,
                    "level": action.level,
                    "wall_start_sec": action.wall_start.sec,
                    "wall_start_nanosec": action.wall_start.nanosec,
                    "wall_end_sec": action.wall_end.sec,
                    "wall_end_nanosec": action.wall_end.nanosec,
                    "elapsed_time": msg.elapsed_time,
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

                # Update tracking
                self._seen_actions[action_key] = status_name

        self._message_count += 1
        if self._message_count % 10 == 0:
            self.get_logger().info(f"Received {self._message_count} messages")

    def _get_overall_status_name(self, status_code):
        """Convert overall status code to name."""
        overall_status_map = {
            0: "IDLE",
            1: "RUNNING",
            2: "SUCCEEDED",
            3: "FAILED",
            4: "CANCELLED",
        }
        return overall_status_map.get(status_code, f"UNKNOWN({status_code})")

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
    # subscriber = PlanExecutionSubscriber(csv_filename="custom_output.csv")
    subscriber = PlanExecutionSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info("Shutting down...")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
